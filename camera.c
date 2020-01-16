#include <stdlib.h>
#include <stdio.h>
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiCLI.h"
#include "RaspiTex.h"
#include "RaspiHelpers.h"
#include "RaspiGPS.h"
#include "RaspiPreview.h"

#include <semaphore.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <string.h>
#include <ctype.h>

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1 
#define MMAL_CAMERA_CAPTURE_PORT 2

//stills format info
//0 implies variable
#define STILLS_FRAME_RATE_NUM 0
#define STILLS_FRAME_RATE_DEN 1

//preview frame rate
#define PREVIEW_FRAME_RATE_NUM 0
#define PREVIEW_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

#define MAX_USER_EXIF_TAGS      32
#define MAX_EXIF_PAYLOAD_LENGTH 128

#define EX_OK 0
#define EX_SOFTWARE 70

/// Amount of time before first image taken to allow settling of
/// exposure etc. in milliseconds.
#define CAMERA_SETTLE_TIME       1000

//central hub of data and parameters
typedef struct 
{
   RASPICOMMONSETTINGS_PARAMETERS common_settings;     /// Common settings
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   char *linkname;                     /// filename of output file
   int frameStart;                     /// First number of frame output counter
   MMAL_FOURCC_T encoding;             /// Encoding to use for the output file.
   //const char *exifTags[MAX_USER_EXIF_TAGS]; /// Array of pointers to tags supplied from the command line
   int numExifTags;                    /// Number of supplied tags
   int enableExifTags;                 /// Enable/Disable EXIF tags in output
   int frameNextMethod;                /// Which method to use to advance to next frame

   RASPIPREVIEW_PARAMETERS preview_parameters;    /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encode
   MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port
}RASPISTILL_STATE;


/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
   RASPISTILL_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

//camera commands
enum
{
   FRAME_NEXT_SINGLE,
   FRAME_NEXT_TIMELAPSE,
   FRAME_NEXT_KEYPRESS,
   FRAME_NEXT_FOREVER,
   FRAME_NEXT_GPIO,
   FRAME_NEXT_SIGNAL,
   FRAME_NEXT_IMMEDIATELY
};


/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPISTILL_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   memset(state, 0, sizeof(*state));

   raspicommonsettings_set_defaults(&state->common_settings);

   state->timeout = -1; // replaced with 5000ms later if unset
   state->linkname = NULL;
   state->frameStart = 0;
   state->camera_component = NULL;
   state->encoder_component = NULL;
   state->preview_connection = NULL;
   state->encoder_connection = NULL;
   state->encoder_pool = NULL;
   state->encoding = MMAL_ENCODING_JPEG;
   state->frameNextMethod = FRAME_NEXT_SINGLE;
   state->numExifTags = 0;
   state->enableExifTags = 1;

   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

static int wait_for_frame(RASPISTILL_STATE *state, int *frame)
{
   static int64_t complete_time = -1;
   int keep_running = 1;

   int64_t current_time = get_microseconds64()/1000;

   if (complete_time == -1)
   {
      complete_time = current_time + state->timeout;
   }
   //if timeout == 0 then continue
   if(current_time >= complete_time && state->timeout != 0)
   {
      keep_running = 0;
   }

   //for development for the option to be frame next single 
   state->frameNextMethod = FRAME_NEXT_SINGLE;

   switch (state->frameNextMethod)
   {
      case FRAME_NEXT_SINGLE :
         // simple timeout for a single capture
         vcos_sleep(state->timeout);
         return 0;
      
      case FRAME_NEXT_KEYPRESS :
      {
         int ch;

         if (state->common_settings.verbose)
            fprintf(stderr, "Press Enter to capture, X then ENTER to exit\n");

         ch = getchar();
         //will wait forever?

         *frame+=1;
         if (ch == 'x' || ch == 'X')
            return 0;
         else
         {
            return keep_running;
         }
      }
      case FRAME_NEXT_IMMEDIATELY :
      {
         // Not waiting, just go to next frame.
         // Actually, we do need a slight delay here otherwise exposure goes
         // badly wrong since we never allow it frames to work it out
         // This could probably be tuned down.
         // First frame has a much longer delay to ensure we get exposure to a steady state
         if (*frame == 0)
            vcos_sleep(CAMERA_SETTLE_TIME);
         else
            vcos_sleep(30);

         *frame+=1;

         return keep_running;
      }

      case FRAME_NEXT_GPIO :
      {
         // Intended for GPIO firing of a capture
         return 0;
      }
      case FRAME_NEXT_SIGNAL :
      {
         // Need to wait for a SIGUSR1 or SIGUSR2 signal
         sigset_t waitset;
         int sig;
         int result = 0;

         sigemptyset( &waitset );
         sigaddset( &waitset, SIGUSR1 );
         sigaddset( &waitset, SIGUSR2 );

         // We are multi threaded because we use mmal, so need to use the pthread
         // variant of procmask to block until a SIGUSR1 or SIGUSR2 signal appears
         pthread_sigmask( SIG_BLOCK, &waitset, NULL );

         if (state->common_settings.verbose)
         {
            fprintf(stderr, "Waiting for SIGUSR1 to initiate capture and continue or SIGUSR2 to capture and exit\n");
         }

         result = sigwait( &waitset, &sig );

         if (result == 0)
         {
            if (sig == SIGUSR1)
            {
               if (state->common_settings.verbose)
                  fprintf(stderr, "Received SIGUSR1\n");
            }
            else if (sig == SIGUSR2)
            {
               if (state->common_settings.verbose)
                  fprintf(stderr, "Received SIGUSR2\n");
               keep_running = 0;
            }
         }
         else
         {
            if (state->common_settings.verbose)
               fprintf(stderr, "Bad signal received - error %d\n", errno);
         }

         *frame+=1;

         return keep_running;
      }
   } // end of switch

   // Should have returned by now, but default to timeout
   return keep_running;
}



/**
 * Allocates and generates a filename based on the
 * user-supplied pattern and the frame number.
 * On successful return, finalName and tempName point to malloc()ed strings
 * which must be freed externally.  (On failure, returns nulls that
 * don't need free()ing.)
 *
 * @param finalName pointer receives an
 * @param pattern sprintf pattern with %d to be replaced by frame
 * @param frame for timelapse, the frame number
 * @return Returns a MMAL_STATUS_T giving result of operation
*/

MMAL_STATUS_T name_photo(char** finalName, char** tempName, char * pattern, int frame)
{
   *finalName = NULL;
   *tempName = NULL;
   if (0 > asprintf(finalName, pattern, frame) ||
         0 > asprintf(tempName, "%s~", *finalName))
   {
      if (*finalName != NULL)
      {
         free(*finalName);
      }
      return MMAL_ENOMEM;    // It may be some other error, but it is not worth getting it right
   }
   return MMAL_SUCCESS;
}


//linked filename to data using gnu opeartions
static void rename_file(RASPISTILL_STATE *state, FILE *output_file,
                        const char *final_filename, const char *temp_filename, int frame)
{
   MMAL_STATUS_T status;

   fclose(output_file);
   vcos_assert(temp_filename != NULL && final_filename != NULL);
   if (0 != rename(temp_filename, final_filename))
   {
      vcos_log_error("Could not rename temp file to: %s; %s",
                     final_filename,strerror(errno));
   }
   if (state->linkname)
   {
      char *use_link;
      char *final_link;
      status = name_photo(&final_link, &use_link, state->linkname, frame);

      // Create hard link if possible, symlink otherwise
      if (status != MMAL_SUCCESS
            || (0 != link(final_filename, use_link)
                &&  0 != symlink(final_filename, use_link))
            || 0 != rename(use_link, final_link))
      {
         vcos_log_error("Could not link as filename: %s; %s",
                        state->linkname,strerror(errno));
      }
      if (use_link) free(use_link);
      if (final_link) free(final_link);
   }
}


int open_filename(RASPISTILL_STATE *state, char *use_filename, char *final_filename, FILE *output_file, int frame, PORT_USERDATA callback_data)
{
   // Open the file
         MMAL_STATUS_T status;
         vcos_assert(use_filename == NULL && final_filename == NULL);
         status = name_photo(&final_filename, &use_filename, state->common_settings.filename, frame);
         if (status  != MMAL_SUCCESS)
         {
            vcos_log_error("Unable to create filenames");
            //goto error;
            exit(1);
         }

         if (state->common_settings.verbose)
            fprintf(stderr, "Opening output file %s\n", final_filename);
         // Technically it is opening the temp~ filename which will be renamed to the final filename

         output_file = fopen(use_filename, "wb");

         if (!output_file)
         {
            // Notify user, carry on but discarding encoded output buffers
            vcos_log_error("%s: Error opening output file: %s\nNo output file will be generated\n", __func__, use_filename);
         }
   
      callback_data.file_handle = output_file;
}
          

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct. encoder_component member set to the created camera_component if successful.
 *
 * @return a MMAL_STATUS, MMAL_SUCCESS if all OK, something else otherwise
 */
static MMAL_STATUS_T create_encoder_component(RASPISTILL_STATE *state)
{
   MMAL_COMPONENT_T *encoder = 0;
   MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to create JPEG encoder component");
      goto error;
   }

   if (!encoder->input_num || !encoder->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("JPEG encoder doesn't have input/output ports");
      goto error;
   }

   encoder_input = encoder->input[0];
   encoder_output = encoder->output[0];

   // We want same format on input and output
   mmal_format_copy(encoder_output->format, encoder_input->format);

   // Specify out output format
   encoder_output->format->encoding = state->encoding;

   encoder_output->buffer_size = encoder_output->buffer_size_recommended;

   if (encoder_output->buffer_size < encoder_output->buffer_size_min)
      encoder_output->buffer_size = encoder_output->buffer_size_min;

   encoder_output->buffer_num = encoder_output->buffer_num_recommended;

   if (encoder_output->buffer_num < encoder_output->buffer_num_min)
      encoder_output->buffer_num = encoder_output->buffer_num_min;

   // Commit the port changes to the output port
   status = mmal_port_format_commit(encoder_output);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set format on video encoder output port");
      goto error;
   }

   // Set the JPEG quality level
   //last value can be satte->quality but for simplicity use 50(on 1-100 scale)
   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, 50);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set JPEG quality");
      goto error;
   }

   // Set the JPEG restart interval
   //not sure what restart_interval(last parameter) shoudl be, using 0
   status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_RESTART_INTERVAL, 0);
   
   //if state->restart_interval != MMAL_SUCCESS
   if (0 && status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to set JPEG restart interval");
      goto error;
   }

   //  Enable component
   status = mmal_component_enable(encoder);

   if (status  != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable video encoder component");
      goto error;
   }

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
   }

   state->encoder_pool = pool;
   state->encoder_component = encoder;

   if (state->common_settings.verbose)
      printf("Encoder component done\n");

   return status;

error:

   if (encoder)
      mmal_component_destroy(encoder);

   return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPISTILL_STATE *state)
{
   // Get rid of any port buffers first
   if (state->encoder_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}



/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   int complete = 0;

   // We pass our file handle and other stuff in via the userdata field.

   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

   if (pData)
   {
      int bytes_written = buffer->length;

      if (buffer->length && pData->file_handle)
      {
         mmal_buffer_header_mem_lock(buffer);

         bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);

         mmal_buffer_header_mem_unlock(buffer);
      }

      // We need to check we wrote what we wanted - it's possible we have run out of storage.
      if (bytes_written != buffer->length)
      {
         vcos_log_error("Unable to write buffer to file - aborting");
         complete = 1;
      }

      // Now flag if we have completed
      if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
         complete = 1;
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status = MMAL_SUCCESS;
      MMAL_BUFFER_HEADER_T *new_buffer;

      new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);

      if (new_buffer)
      {
         status = mmal_port_send_buffer(port, new_buffer);
      }
      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }

   if (complete)
      vcos_semaphore_post(&(pData->complete_semaphore));
}




int create_camera_component(RASPISTILL_STATE *state)
{
	MMAL_COMPONENT_T *camera = 0;
	MMAL_ES_FORMAT_T *format;
	MMAL_PORT_T *preview_port = NULL;
	MMAL_PORT_T *video_port = NULL;
	MMAL_PORT_T *still_port = NULL;

	MMAL_STATUS_T operation_status;

	//create the component
	operation_status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

	if (operation_status != MMAL_SUCCESS)
	{
		printf("Unable to create camera component : error code : %d\n", operation_status );
		exit(1);
	}

	//seems like I need the stereoscopic mode for good photos, possibility I don't need them.
   operation_status = raspicamcontrol_set_stereo_mode(camera->output[0], &state->camera_parameters.stereo_mode);
   operation_status += raspicamcontrol_set_stereo_mode(camera->output[1], &state->camera_parameters.stereo_mode);
   operation_status += raspicamcontrol_set_stereo_mode(camera->output[2], &state->camera_parameters.stereo_mode);

   if (operation_status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set stereo mode : error %d", operation_status);
      exit(1);
      //goto error;
   }
   
   MMAL_PARAMETER_INT32_T camera_num =
   {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

   operation_status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (operation_status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", operation_status);
      exit(1);
//      goto error;
   }

	if (!camera->output_num)
	{
		operation_status = MMAL_ENOSYS;
		printf("Camera has no output ports\n");
		exit(1);
	}

	//may want to set sensor mode for sport shot or night shot
   // operation_status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

   // if (operation_status != MMAL_SUCCESS)
   // {
   //    vcos_log_error("Could not set sensor mode : error %d", operation_status);
   //    exit(1);
   // }

	//set up ports
	preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	//video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	//enabling camera and setup control callback function
	operation_status = mmal_port_enable(camera->control, default_camera_control_callback);

	if(operation_status != MMAL_SUCCESS) 
	{
		printf("Error enabling control port: error code:%d \n Destroying camera",operation_status);
		mmal_component_destroy(camera);
		exit(1);
	}	

   //  set up the camera configuration
   MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
   {
      { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
      .max_stills_w = state->common_settings.width,
      .max_stills_h = state->common_settings.height,
      .stills_yuv422 = 0,
      .one_shot_stills = 1,
      .max_preview_video_w = state->preview_parameters.previewWindow.width,
      .max_preview_video_h = state->preview_parameters.previewWindow.height,
      .num_preview_video_frames = 3,
      .stills_capture_circular_buffer_height = 0,
      .fast_preview_resume = 0,
      .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
   };

	//since fullResPreview setting is ommitted preview will not be as nice as actual photo

	//mmal_port_parameter_set commits parameter changes to the port, 
	//shoudl eb done to all ports(encoder, camera) when variables are being set
	mmal_port_parameter_set(camera->control, &cam_config.hdr);

	//apply the paramters
	raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

	//enable preview port
	operation_status = enable_port(&state, camera, preview_port);
	//enable still/photo
	operation_status = enable_port(&state, camera, still_port);

	/* Enable component */
   operation_status = mmal_component_enable(camera);

   state->camera_component = camera;

   if (operation_status == MMAL_SUCCESS)
      fprintf(stderr, "Camera component done\n");

   return operation_status;

// error:

//    if (camera)
//       mmal_component_destroy(camera);

//    return status;
}


/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPISTILL_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}


int enable_port(RASPISTILL_STATE *state, MMAL_COMPONENT_T *camera, MMAL_PORT_T *port)
{
   MMAL_STATUS_T status;
   MMAL_ES_FORMAT_T *format;

	format = port->format;

	if(state->camera_parameters.shutter_speed > 6000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
			{ 50, 1000 }, {166, 1000}
		};
		mmal_port_parameter_set(port, &fps_range.hdr);
	}
	else if(state->camera_parameters.shutter_speed > 1000000)
	{
		MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
			{ 166, 1000 }, {999, 1000}
		};
		mmal_port_parameter_set(port, &fps_range.hdr);
	}

	// could use Fullrespreview but more frames is betterUse a full FOV 4:3 mode

	if(port == camera->output[MMAL_CAMERA_PREVIEW_PORT])
	{

      format->encoding = MMAL_ENCODING_OPAQUE;   
		format->encoding_variant = MMAL_ENCODING_I420;
   	format->es->video.width = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.width, 32);
	   format->es->video.height = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.height, 16);
   	format->es->video.crop.x = 0;
	   format->es->video.crop.y = 0;
		format->es->video.crop.width = state->preview_parameters.previewWindow.width;
		format->es->video.crop.height = state->preview_parameters.previewWindow.height;
		format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
		format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;
		status = mmal_port_format_commit(port);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("camera viewfinder format couldn't be set");
			goto error;
		}
		else
		{
			fprintf(stderr, "Created Preview component\n");
		}

	}
	else if(port == camera->output[MMAL_CAMERA_CAPTURE_PORT])
	{
      //for some reason using the preview parameters works but not the common settings ones
      format->es->video.width = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.width, 32);
	   format->es->video.height = VCOS_ALIGN_UP(state->preview_parameters.previewWindow.height, 16);
      // format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
      // format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   	format->es->video.crop.x = 0;
	   format->es->video.crop.y = 0;
		// format->es->video.crop.width = state->common_settings.width;
		// format->es->video.crop.height = state->common_settings.height;

   	format->es->video.crop.width = state->preview_parameters.previewWindow.width;
		format->es->video.crop.height = state->preview_parameters.previewWindow.height;

		format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
		format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;

		status = mmal_port_format_commit(port);

		if (port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      		port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("camera still format couldn't be set");
			goto error;
		}
	}
	return 0;
error:

if (camera)
   mmal_component_destroy(camera);

return status;
}

//might not work tbh
// int take_single_photo(MMAL_STATUS_T operation_status, MMAL_PORT_T *still_port)
// {
// 	//capture
// 	operation_status = mmal_port_parameter_set_boolean(still_port, MMAL_PARAMETER_CAPTURE, 1);
// 	if (operation_status != MMAL_SUCCESS)
// 	{
// 		fprintf(stderr, "Failed to capture\n");
// 		exit(1);
// 	}
// 	fprintf(stderr, "camera initialized, picture taken?");
// 	char file_name[10] = "photo1.jpg";
// 	char* final_name; 
// 	if(name_photo(&final_name, file_name, -1) != MMAL_SUCCESS)
// 	{
// 		fprintf(stderr, "Unable to write to filename\n");
// 	}
// 	fprintf(stderr, "here\n");
// 	return 0;
// }




int main()
{
   RASPISTILL_STATE state;
   int exit_code = EX_OK;

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_preview_port = NULL;
   //MMAL_PORT_T *camera_video_port = NULL;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *preview_input_port = NULL;
   MMAL_PORT_T *encoder_input_port = NULL;
   MMAL_PORT_T *encoder_output_port = NULL;


//what is thiss??
   bcm_host_init();

      // Register our application with the logging system
      //what is this??
      // some threading thing, probably will replace with pthread stuff
   vcos_log_register("RaspiStill", VCOS_LOG_CATEGORY);


//not sure what the signal stuff is
   signal(SIGINT, default_signal_handler);

   // Disable USR1 and USR2 for the moment - may be reenabled if go in to signal capture mode
   signal(SIGUSR1, SIG_IGN);
   signal(SIGUSR2, SIG_IGN);
   default_status(&state);


   if (state.timeout == -1)
      state.timeout = 5000;

   // Setup for sensor specific parameters
   get_sensor_defaults(state.common_settings.cameraNum, state.common_settings.camera_name,
                       &state.common_settings.width, &state.common_settings.height);


   //create camera, preview and encoder component
   if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create camera component", __func__);
      exit_code = EX_SOFTWARE;
   }
   if ((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create preview component", __func__);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }
   if ((status = create_encoder_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create encode component", __func__);
      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }

   PORT_USERDATA callback_data;

   fprintf(stderr, "Starting component connection stage\n");

   camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
   //camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
   camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
   encoder_input_port  = state.encoder_component->input[0];
   encoder_output_port = state.encoder_component->output[0];

   fprintf(stderr, "Connecting camera preview port to video render.\n");

   // Note we are lucky that the preview and null sink components use the same input port
   // so we can simple do this without conditionals
   preview_input_port  = state.preview_parameters.preview_component->input[0];

   // Connect camera to preview (which might be a null_sink if no preview required)
   status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);

   VCOS_STATUS_T vcos_status;

   if (status == MMAL_SUCCESS)
   {
         fprintf(stderr, "Connecting camera stills port to encoder input port\n");

      // Now connect the camera to the encoder
      status = connect_ports(camera_still_port, encoder_input_port, &state.encoder_connection);

      if (status != MMAL_SUCCESS)
      {
         vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
         exit(1);
      }
   }
   else
   {
      exit(1);
   }
   


//I believe this is where the photo data will be stored for useage

   callback_data.file_handle = NULL;
   callback_data.pstate = &state;

   //threading stuff tah will need to be researched
   vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
   vcos_assert(vcos_status == VCOS_SUCCESS);


//below is the operation of the raspistill functions
   int frame, keep_looping = 1;
   FILE *output_file = NULL;
   char *use_filename = NULL;      // Temporary filename while image being written
   char *final_filename = NULL;    // Name that file gets once writing complete

//wish there was comments for what frame is exactly
   frame = state.frameStart - 1;

// need to get frame or something
   wait_for_frame(&state, &frame);

   // need to open the filename so data can be allocated to it
   //possibly add functionality to add date to name so we can keep making new photo files
   //use a set filename for now for testing

   char testing_photo_name[10] =  "photo.jpeg";
   int len = strlen(testing_photo_name);
   state.common_settings.filename = malloc(len + 10); // leave enough space for any timelapse generated changes to filename
   vcos_assert(state.common_settings.filename);
   if (state.common_settings.filename)
      strncpy(state.common_settings.filename, testing_photo_name, len+1);

   open_filename(&state, use_filename, final_filename, output_file, frame, callback_data);

   if (!output_file)
   {
        fprintf(stderr, "No output file avaliable");
        exit(1); 
   }

   mmal_port_parameter_set_boolean(state.encoder_component->output[0], MMAL_PARAMETER_EXIF_DISABLE, 1);

   // There is a possibility that shutter needs to be set each loop. may not be necessary
   if (mmal_status_to_int(mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, state.camera_parameters.shutter_speed)) != MMAL_SUCCESS)
      vcos_log_error("Unable to set shutter speed");


   // Enable the encoder output port
   encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

   if (state.common_settings.verbose)
      fprintf(stderr, "Enabling encoder output port\n");

   // Enable the encoder output port and tell it its callback function
   status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);

   // Send all the buffers to the encoder output port
   int num = mmal_queue_length(state.encoder_pool->queue);

   for (int q=0; q<num; q++)
   {
      MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);

      if (!buffer)
         vcos_log_error("Unable to get a required buffer %d from pool queue", q);

      if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
         vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
   }

   //capture?
   if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to start capture", __func__);
   }
   else
   {
      // Wait for capture to complete
      // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
      // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
      vcos_semaphore_wait(&callback_data.complete_semaphore);
      if (state.common_settings.verbose)
         fprintf(stderr, "Finished capture %d\n", frame);
   }

   // Ensure we don't die if get callback with no open file
  
   callback_data.file_handle = NULL;
  
   rename_file(&state, output_file, final_filename, use_filename, frame);
   
   status = mmal_port_disable(encoder_output_port);
   
   if (use_filename)
   {
      free(use_filename);
      use_filename = NULL;
   }
   if (final_filename)
   {
      free(final_filename);
      final_filename = NULL;
   }
   
   vcos_semaphore_delete(&callback_data.complete_semaphore);
   fprintf(stderr,"Done");

	return 0;

}
     