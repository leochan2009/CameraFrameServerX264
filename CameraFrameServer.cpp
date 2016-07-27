#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include "igtl_header.h"
#include "igtl_video.h"


#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlVideoMessage.h"
#include "igtlServerSocket.h"
#include "igtlMultiThreader.h"
extern "C" {
#include "stdint.h"
#include "x264.h"
#include "x264Local.c"
}

void* ThreadFunction(void* ptr);
int   SendVideoData(igtl::Socket::Pointer& socket, igtl::VideoMessage::Pointer& videoMsg);

typedef struct {
  cv::VideoCapture cap;
  int   nloop;
  igtl::MutexLock::Pointer glock;
  igtl::Socket::Pointer socket;
  int   interval;
  int   stop;
  bool  useCompression;
} ThreadData;

std::string     videoFile = "";

void CalculateFrameRate(cv::VideoCapture cap);

int main(int argc, char* argv[])
{
  //------------------------------------------------------------
  // Parse Arguments
  
  if (argc != 2) // check number of arguments
  {
    // If not correct, print usage
    std::cerr << "Usage: " << argv[0] << " <port>"    << std::endl;
    std::cerr << "    <port>     : Port # (18944 in Slicer default)"   << std::endl;
    exit(0);
  }
  
  int    port     = atoi(argv[1]);
  
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);
  
  if (r < 0)
  {
    std::cerr << "Cannot create a server socket." << std::endl;
    exit(0);
  }
  igtl::MultiThreader::Pointer threader = igtl::MultiThreader::New();
  igtl::MutexLock::Pointer glock = igtl::MutexLock::New();
  ThreadData td;
  cv::VideoCapture cap;
  cap.open(0);
  CalculateFrameRate(cap);
  cap.release();
  while(1){
    //------------------------------------------------------------
    // Waiting for Connection
    int threadID = -1;
    igtl::Socket::Pointer socket;
    socket = serverSocket->WaitForConnection(1000);
    
    if (socket.IsNotNull()) // if client connected
    {
      std::cerr << "A client is connected." << std::endl;
      
      // Create a message buffer to receive header
      igtl::MessageHeader::Pointer headerMsg;
      headerMsg = igtl::MessageHeader::New();
      //------------------------------------------------------------
      // loop
      for (;;)
      {
        // Initialize receive buffer
        headerMsg->InitPack();
        
        // Receive generic header from the socket
        int rs = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
        if (rs == 0)
        {
          if (threadID >= 0)
          {
            td.stop = 1;
            threader->TerminateThread(threadID);
            threadID = -1;
          }
          std::cerr << "Disconnecting the client." << std::endl;
          td.socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
          socket->CloseSocket();
          break;
        }
        if (rs != headerMsg->GetPackSize())
        {
          continue;
        }
        
        // Deserialize the header
        headerMsg->Unpack();
        
        // Check data type and receive data body
        if (strcmp(headerMsg->GetDeviceType(), "STT_VIDEO") == 0)
        {
          std::cerr << "Received a STT_VIDEO message." << std::endl;
          
          igtl::StartVideoDataMessage::Pointer startVideoMsg;
          startVideoMsg = igtl::StartVideoDataMessage::New();
          startVideoMsg->SetMessageHeader(headerMsg);
          startVideoMsg->AllocatePack();
          
          socket->Receive(startVideoMsg->GetPackBodyPointer(), startVideoMsg->GetPackBodySize());
          int c = startVideoMsg->Unpack(1);
          if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
          {
            if (!cap.isOpened())
            {
              cap.open(0);
            }
            td.interval = startVideoMsg->GetResolution();
            td.glock    = glock;
            td.socket   = socket;
            td.stop     = 0;
            td.cap = cap;
            td.useCompression = startVideoMsg->GetUseCompress();
            threadID    = threader->SpawnThread((igtl::ThreadFunctionType) &ThreadFunction, &td);
          }
        }
        else if (strcmp(headerMsg->GetDeviceType(), "STP_VIDEO") == 0)
        {
          socket->Skip(headerMsg->GetBodySizeToRead(), 0);
          std::cerr << "Received a STP_VIDEO message." << std::endl;
          if (threadID >= 0)
          {
            td.stop  = 1;
            threader->TerminateThread(threadID);
            threadID = -1;
            std::cerr << "Disconnecting the client." << std::endl;
            cap.release();
            td.socket = NULL;  // VERY IMPORTANT. Completely remove the instance.
            socket->CloseSocket();
          }
          break;
        }
        else
        {
          std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
          socket->Skip(headerMsg->GetBodySizeToRead(), 0);
        }
      }
    }
      //memcpy(pYUVBuf, yuvImg.data, bufLen*sizeof(unsigned char));
      //fwrite(pYUVBuf, bufLen*sizeof(unsigned char), 1, pFileOut);
  }

  return 1;
}

void CalculateFrameRate(cv::VideoCapture cap)
{
  double fps = cap.get(CV_CAP_PROP_FPS);
  // If you do not care about backward compatibility
  // You can use the following instead for OpenCV 3
  // double fps = video.get(CAP_PROP_FPS);
  std::cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps << std::endl;
  // Number of frames to capture
  int num_frames = 60;
  
  // Start and end times
  time_t start, end;
  
  // Variable for storing video frames
  cv::Mat frame;
  
  std::cout << "Capturing " << num_frames << " frames" << std::endl ;
  
  // Start time
  time(&start);
  
  // Grab a few frames
  for(int i = 0; i < num_frames; i++)
  {
    cap >> frame;
  }
  
  // End Time
  time(&end);
  
  // Time elapsed
  double seconds = difftime (end, start);
  std::cout << "Time taken : " << seconds << " seconds" << std::endl;
  
  // Calculate frames per second
  fps  = num_frames / seconds;
  std::cout << "Estimated frames per second : " << fps << std::endl;
  
  // Release video
  cap.release();
  
}


void* ThreadFunction(void* ptr)
{
  //------------------------------------------------------------
  // Get thread information
  igtl::MultiThreader::ThreadInfo* info =
  static_cast<igtl::MultiThreader::ThreadInfo*>(ptr);
  
  //int id      = info->ThreadID;
  //int nThread = info->NumberOfThreads;
  ThreadData* td = static_cast<ThreadData*>(info->UserData);
  
  //------------------------------------------------------------
  // Get user data
  igtl::MutexLock::Pointer glock = td->glock;
  long interval = td->interval;
  std::cerr << "Interval = " << interval << " (ms)" << std::endl;
  //long interval = 1000;
  //long interval = (id + 1) * 100; // (ms)
  
  igtl::Socket::Pointer& socket = td->socket;
  
  //------------------------------------------------------------
  // Allocate TrackingData Message Class
  //
  // NOTE: TrackingDataElement class instances are allocated
  //       before the loop starts to avoid reallocation
  //       in each image transfer.
  x264_param_t* param;
  cli_opt_t* opt;
  x264_t *h = NULL;
  h = x264_encoder_open( param );
  if (h != NULL)
  {
    int argc = 2;
    char *argv[2] = {"--qp 0","--crf 24"};
    x264_picture_t pic;
    cli_pic_t cli_pic;
    x264_picture_clean(&pic);
    int picWidth = 1024, picHeight = 720;
    x264_picture_alloc(&pic, X264_CSP_BGR, picWidth, picHeight);
    pic.img.i_plane = 3;
    const cli_pulldown_t *pulldown = NULL; // shut up gcc
    
    int     i_frame = 0;
    int     i_frame_output = 0;
    int64_t i_end, i_previous = 0, i_start = 0;
    int64_t i_file = 0;
    int     i_frame_size;
    int64_t last_dts = 0;
    int64_t prev_dts = 0;
    int64_t first_dts = 0;
#   define  MAX_PTS_WARNING 3 /* arbitrary */
    int     pts_warning_cnt = 0;
    int64_t largest_pts = -1;
    int64_t second_largest_pts = -1;
    int64_t ticks_per_frame;
    double  duration;
    double  pulldown_pts = 0;
    int     retval = 0;
    
    opt->b_progress &= param->i_log_level < X264_LOG_DEBUG;
    
    /* set up pulldown */
    if( opt->i_pulldown && !param->b_vfr_input )
    {
      param->b_pulldown = 1;
      param->b_pic_struct = 1;
      pulldown = &pulldown_values[opt->i_pulldown];
      param->i_timebase_num = param->i_fps_den;
      param->i_timebase_den = param->i_fps_num * pulldown->fps_factor;
    }

    parse( argc, argv, param, opt ) ;
    x264_encoder_parameters( h, param );
    
    /* ticks/frame = ticks/second / frames/second */
    ticks_per_frame = (int64_t)param->i_timebase_den * param->i_fps_den / param->i_timebase_num / param->i_fps_num;
    ticks_per_frame = X264_MAX( ticks_per_frame, 1 );
    
    if (td->cap.isOpened())
    {
      while (!td->stop)
      {
        cv::Mat frame;
        td->cap >> frame;
        if(frame.empty()){
          std::cerr<<"frame is empty"<<std::endl;
          break;
        }
        cv::imshow("", frame);
        cv::waitKey(10);
        if (!td->useCompression)
        {
          cv::Mat rgbImg;
          cv::cvtColor(frame, rgbImg, CV_BGR2RGB);
          igtl::VideoMessage::Pointer videoMsg;
          videoMsg = igtl::VideoMessage::New();
          videoMsg->SetDeviceName("Video");
          videoMsg->SetBitStreamSize(picWidth*picHeight*3);
          videoMsg->AllocateScalars();
          videoMsg->SetScalarType(videoMsg->TYPE_UINT32);
          videoMsg->SetEndian(igtl_is_little_endian()==true?2:1); //little endian is 2 big endian is 1
          videoMsg->SetWidth(picWidth);
          videoMsg->SetHeight(picHeight);
          memcpy(videoMsg->GetPackFragmentPointer(2), rgbImg.data, picWidth*picHeight*3);
          videoMsg->Pack();
          glock->Lock();
          for (int i = 0; i < videoMsg->GetNumberOfPackFragments(); i ++)
          {
            socket->Send(videoMsg->GetPackFragmentPointer(i), videoMsg->GetPackFragmentSize(i));
          }
          glock->Unlock();
          igtl::Sleep(interval);
        }
        else{
          pic.i_pts = i_frame;
          pic.img.i_stride[0] = pic.img.i_stride[1] = pic.img.i_stride[2] = picWidth;
          pic.img.plane[0] = frame.data;
          pic.img.plane[1] = pic.img.plane[0] + picWidth*picHeight;
          pic.img.plane[2] = pic.img.plane[1] + picWidth*picHeight;
          if( !param->b_vfr_input )
            pic.i_pts = i_frame;
          
          if( opt->i_pulldown && !param->b_vfr_input )
          {
            pic.i_pic_struct = pulldown->pattern[ i_frame % pulldown->mod ];
            pic.i_pts = (int64_t)( pulldown_pts + 0.5 );
            pulldown_pts += pulldown_frame_duration[pic.i_pic_struct];
          }
          else if( opt->timebase_convert_multiplier )
            pic.i_pts = (int64_t)( pic.i_pts * opt->timebase_convert_multiplier + 0.5 );
          
          if( pic.i_pts <= largest_pts )
          {
            if( cli_log_level >= X264_LOG_DEBUG || pts_warning_cnt < MAX_PTS_WARNING )
              x264_cli_log( "x264", X264_LOG_WARNING, "non-strictly-monotonic pts at frame %d (%"PRId64" <= %"PRId64")\n",
                           i_frame, pic.i_pts, largest_pts );
            else if( pts_warning_cnt == MAX_PTS_WARNING )
              x264_cli_log( "x264", X264_LOG_WARNING, "too many nonmonotonic pts warnings, suppressing further ones\n" );
            pts_warning_cnt++;
            pic.i_pts = largest_pts + ticks_per_frame;
          }
          
          second_largest_pts = largest_pts;
          largest_pts = pic.i_pts;
          if( opt->qpfile )
            parse_qpfile( opt, &pic, i_frame + opt->i_seek );
          
          x264_picture_t pic_out;
          x264_nal_t *nal;
          int i_nal;
          int i_frame_size = 0;
          i_frame_size = x264_encoder_encode( h, &nal, &i_nal, &pic, &pic_out );
          
          if( i_frame_size )
          i_frame++;
          if(i_frame_size > 0)
          {
            last_dts = pic_out.i_dts;
            // 1. contain SHA encryption, could be removed, 2. contain the digest message could be as CRC
            //UpdateHashFromFrame (info, &ctx);
            //---------------
            igtl::VideoMessage::Pointer videoMsg;
            videoMsg = igtl::VideoMessage::New();
            videoMsg->SetDeviceName("Video");
            videoMsg->SetBitStreamSize(i_frame_size);
            videoMsg->AllocateScalars();
            videoMsg->SetScalarType(videoMsg->TYPE_UINT32);
            videoMsg->SetEndian(igtl_is_little_endian()==true?2:1); //little endian is 2 big endian is 1
            videoMsg->SetWidth(picWidth);
            videoMsg->SetHeight(picHeight);
            memcpy(videoMsg->GetPackFragmentPointer(2),(nal[0].p_payload), i_frame_size);
            videoMsg->Pack();
            glock->Lock();
            
            for (int i = 0; i < videoMsg->GetNumberOfPackFragments(); i ++)
            {
              socket->Send(videoMsg->GetPackFragmentPointer(i), videoMsg->GetPackFragmentSize(i));
            }
            glock->Unlock();
            igtl::Sleep(interval);
          }
        }
      }
    }
  }
  x264_encoder_close( h );
  return NULL;
}

