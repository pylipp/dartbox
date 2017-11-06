/*
 * rosserial Service Server
 */

#include <ros.h>
#include <rosserial_arduino/Test.h>

ros::NodeHandle  nh;
using rosserial_arduino::Test;

int i = 0;
void callback(const Test::Request & req, Test::Response & res){
  // simulate user input according to Game.run() and Player.play()
  if (i == 0)
    res.output = "1";
  else if (i == 1)
    res.output = "501";
  else if (i == 2)
    res.output = "Peter";
  else if (i == 3)
    res.output = "1";
  else if (i == 4)
    res.output = "180";
  else if (i == 5)
    res.output = "180";
  else if (i == 6) {
    res.output = "141";
    i = -1;  // reset
  }
  i++;
}

ros::ServiceServer<Test::Request, Test::Response> server("get_input", &callback);

void setup()
{
  nh.initNode();
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
  delay(10);
}
