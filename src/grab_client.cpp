#include "rclcpp/rclcpp.hpp"
#include "grab_interface/srv/grab_srv_data.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>

using namespace std::chrono_literals;

int scanKeyboard()
{
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
  new_settings = stored_settings;           //
  new_settings.c_lflag &= (~ICANON);        //
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(STDIN_FILENO,TCSANOW,&new_settings); //

  in = getchar();

  tcsetattr(STDIN_FILENO,TCSANOW,&stored_settings);
  return in;
}
static struct termios initial_settings, new_settings;
static int peek_character = -1;

void init_keyboard()
{
	tcgetattr(0,&initial_settings);
	new_settings = initial_settings;
	new_settings.c_lflag |= ICANON;
	new_settings.c_lflag |= ECHO;
	new_settings.c_lflag |= ISIG;
	new_settings.c_cc[VMIN] = 1;
	new_settings.c_cc[VTIME] = 0;
	tcsetattr(0, TCSANOW, &new_settings);
}
 
void close_keyboard()
{
	tcsetattr(0, TCSANOW, &initial_settings);
}
 
int kbhit()
{
	unsigned char ch;
	int nread;
 
	if (peek_character != -1) return 1;
	new_settings.c_cc[VMIN]=0;
	tcsetattr(0, TCSANOW, &new_settings);
	nread = read(0,&ch,1);
	new_settings.c_cc[VMIN]=1;
	tcsetattr(0, TCSANOW, &new_settings);
	if(nread == 1) 
	{
		peek_character = ch;
		return 1;
	}
	return 0;
}
 
int readch()
{
	char ch;
 
	if(peek_character != -1) 
	{
		ch = peek_character;
		peek_character = -1;
		return ch;
	}
	read(0,&ch,1);
	return ch;
}

int main(int argc, char **argv)
{
  init_keyboard();
  rclcpp::init(argc, argv);

  if (argc != 1) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "only one type is valid");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grab_client");
  rclcpp::Client<grab_interface::srv::GrabSrvData>::SharedPtr client =
    node->create_client<grab_interface::srv::GrabSrvData>("grab_srvcli");

  auto request = std::make_shared<grab_interface::srv::GrabSrvData::Request>();
  //request->grab_type = atoll(argv[1]);
  while(1)
  {
    //request->grab_type = scanKeyboard();
    kbhit();
    request->grab_type = readch();
    auto result = client->async_send_request(request);

    if(request->grab_type == 'q')
    {
      break;
    }

  }
   
  // while (!client->wait_for_service(1s)) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
  //     return 0;
  //   }
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  // }

  // Wait for the result.
  
  // if (rclcpp::spin_until_future_complete(node, result) ==
  //   rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success: %ld", result.get()->if_success);
  // } else {
  //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service grab");
  // }
  
  rclcpp::shutdown();
  return 0;
}
