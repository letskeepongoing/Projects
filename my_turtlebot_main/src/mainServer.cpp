#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <my_turtlebot_main/dir_out.h>
#include <my_turtlebot_main/odom_array.h>
#include <vector>

class main_turtle{
  protected:
  int rate_hz;
  int state, cycles, final_el, dir;;
  float time_limit; 
  bool action_active, finish;
  float x, y, hypt;
  float constant;
  
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub_read; //this reads the odom
  ros::Subscriber sense; //this reads the laser 
  ros::Rate *rate_;
  geometry_msgs::Twist var;
  nav_msgs::Odometry latest_odometry;
  std_msgs::String direction;
  ros::ServiceServer turtle_main;
  my_turtlebot_main::odom_array odoms;

  
  public:
    main_turtle(){
    this->state = 0;
    this->rate_hz = 20;
    this->constant = 8.4; 
    this->time_limit = 200;
    this->action_active = false;
    this->finish = false;
    this->direction.data ="straight";
    
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    sense = nh.subscribe<sensor_msgs::LaserScan>("/kobuki/laser/scan",100,&main_turtle::readscan,this);
    sub_read = nh.subscribe<nav_msgs::Odometry>("odom",100,&main_turtle::giveOdom,this);
    
    turtle_main = nh.advertiseService("/escaping_maze", &main_turtle::readScanner, this);
    
    rate_= new ros::Rate(rate_hz);
    cycles = (int)time_limit * rate_hz; 
  }
  
  ~main_turtle(){}
  
 void waste_times(){
        rate_->sleep();
    }
  
 void waste_time(){ //well base the sleep off of state
     float turn = 3; //turn for 3 seconds?
     if (state = 1){
      while (turn > 0){
        turn -= 1/(float)rate_hz;
        rate_->sleep();
      }
   }
  }
  
 void findDir(){
   switch (dir){
    case 0:
    direction.data = "straight";
    break;
    case 1:
    direction.data = "left";
    break;
    case 2:
    direction.data = "right";
    break;
   }
  }
  
 void giveOdom(const nav_msgs::Odometry::ConstPtr& _msg){ //msg is a pointer 
  
  
            this->latest_odometry.header = _msg->header;
            this->latest_odometry.child_frame_id = _msg->child_frame_id;
            this->latest_odometry.pose = _msg->pose;

            
            if (this->action_active)
            {
                this->odoms.result_odom_array.push_back(this->latest_odometry); // the result is already an odom matrix so I just put it in here.
            }
 }
 
 bool are_we_out_of_maze() {
            bool result = false;
            
            // This is a security mesure to guarantee that the saved element in the vector is 
            // what we use for calculating.
            
            if (!this->odoms.result_odom_array.empty()) 
            {
                nav_msgs::Odometry last_odom = this->odoms.result_odom_array.back(); //.back() pulls the last element in the array;
                float last_x = last_odom.pose.pose.position.x;
                float last_y = last_odom.pose.pose.position.y;
                
                float hypt = sqrt(pow(last_x,2)+pow(last_y,2));
                // Because the robot starts at the origin we dont have to save the init position
                // To then compare with the latest.
                if (hypt >= this->constant ){
                    ROS_WARN("SUCCESS Distance from origin %f >= %f", hypt, this->constant);
                    result = true;
                }else
                {
                    ROS_ERROR("NOTYET Distance from origin %f >= %f", hypt, this->constant);
                }
            }else
            {
                ROS_ERROR("Not data yet inside this->result_.result_odom_array");
            }
            
            
            return result;
        } //return true or false depending on odom

 void readscan(const sensor_msgs::LaserScan::ConstPtr& msg){

  if(action_active = true){ //this isjust to start all the actions at once //all the movement is right here 
  
  if (this->time_limit > 0 && this->finish == false ){
  
    
    this->finish =are_we_out_of_maze();
    
   if(msg->ranges[360] < 1.4  && msg->ranges[0] > 1){
     var.linear.x = 0;
     var.angular.z = -0.2; //turn for 3.6 seconds
     ROS_INFO("second case %f" ,msg->ranges[360]);
     state = 1;
   }
   else if (msg->ranges[360] < 1 && msg->ranges[719] >1){
    var.linear.x = 0;
     var.angular.z = 0.2; //turn for 3.6 seconds
     ROS_INFO("third case %f" ,msg->ranges[360]);
     state = 1;
   }
   else {
    var.linear.x = .2;
    var.angular.z = 0;
    ROS_INFO("This is the sensor reading the right %f", msg->ranges[0]);
   }
  }
    else { // basically when finish become true we stop
    var.linear.x = 0;
    var.angular.z=0;
    this->move_around();
    if(time_limit > 0){
     ROS_INFO("WE HAVE ESCAPED THE MAZE");
    }
    else {
        ROS_INFO("WE HAVE RUN OUT OF TIME");
    }
    }
  }
    
 }
 
 
 void move_around(){
    pub.publish(var);
  } //publish var 
  
 
  
  bool readScanner(my_turtlebot_main::dir_out::Request &req, 
                    my_turtlebot_main::dir_out::Response &res){
   findDir(); //change dir based on sensor readings
   res.direction = direction.data; //output string 
   ROS_INFO("We are going %s", res.direction.c_str()); //.toString()
   
   
   if (finish){
       res.movement = false;
   }
   else{
   res.movement = true;
   }
   
   action_active = true; 
   
   //success = true
   
   return res.movement; 
  }
  
};


int main(int argc, char** argv) {
    
    ros::init(argc,argv, "main_turtle");
    main_turtle waddler;
    
    
    while (ros::ok()){
      waddler.move_around();
      waddler.waste_time();
      ros::spinOnce();
    }
    return 0;
}
