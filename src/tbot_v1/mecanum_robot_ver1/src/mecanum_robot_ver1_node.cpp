/*
 *  mecanum_robot_ver1_node.cpp
 * 
 *  Created on: July. 11, 2020
 *      Author: cdi
 */

#include "vesc_control.h"
#include "tf/tf.h"

// Settings
#define VESC_ID_0				0
#define VESC_ID_1				1
#define VESC_ID_2				2
#define VESC_ID_3				3
#define VESC_ID_4				4
#define CAN_FORWARD_OFF			0
#define CAN_FORWARD_ON			1
#define BRAKE_CURRENT			10.
#define BRAKE_THRESHOLD			8.	// bigger than (BRAKE_CURRENT/2)

// COMM_SET Types
#define COMM_SET_DUTY			5
#define COMM_SET_CURRENT		6
#define COMM_SET_CURRENT_BRAKE	7
#define COMM_SET_RPM			8
#define COMM_SET_POS			9
#define COMM_SET_HANDBRAKE		10
#define COMM_SET_DPS			38

// Conversions
#define RAD2DEG         		180.0/M_PI  // radian to deg
#define RPS2DPS					RAD2DEG	

// Uncomment this only when you want to see the below infomations.
//#define PRINT_SENSOR_CORE
//#define PRINT_SENSOR_CUSTOMS

void TeleopInput::keyboardCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	/*
	//ROS_INFO("lin x:%.2f, y:%.2f, z:%.2f", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->linear.z);
	//ROS_INFO("ang x:%.2f, y:%.2f, z:%.2f", cmd_vel->angular.x, cmd_vel->angular.y, cmd_vel->angular.z);
	dps[0] = cmd_vel->linear.x*2.;//cmd_vel->linear.x*200.;
	speed[0] = cmd_vel->angular.z*1.;
	if(cmd_vel->angular.z < 0) {
		enable.data = false;
		dps[0] = 0.;
		speed[0] = 0.;
	}
	else 
	{
		startTime = ros::Time::now();
		enable.data = true;
	}
	*/
}

//
void TrapezoidalVelProfile::GenProfile(float v_ref, float *vout)
{
	float da = 0;
	float dv = 0;

	// Profile Deg
	if(v_ref == *vout) {
		dv = 0;
	}
	else {
		da = (v_ref - *vout)/dt;
		if(fabs(da) >= (double)Amax) {
			if(da>0) da = Amax;
			else 	 da = -Amax;
		}
	}
	dv = da*dt;
	*vout += dv;

	//ROS_INFO("dv:%.2f", dv);
}

void TeleopInput::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	static int joy_cont_mode;
	double joy_cmd_forward = 0;
	double joy_cmd_lateral = 0;
	double joy_cmd_steering = 0;
	double joy_cmd_elevation = 0;
	double joy_cmd_hit = 0;
	double joy_cmd_spin = 0;
	static double launcher_lower_speed = 0;
	static double launcher_upper_speed = 0;
	static std_msgs::Bool feeder;

	std::string joy_dev_name = joy->header.frame_id;
	if(joy_dev_name == "/dev/input/js3")	
	{
		//ROS_INFO("joy device: /dev/input/js0 - drone remote");
		//ROS_INFO("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f", joy->axes[0], joy->axes[1], joy->axes[2], joy->axes[3], joy->axes[4], joy->axes[5]);
		//ROS_INFO("%d, %d", joy->buttons[0], joy->buttons[1]);

		joy_cmd_forward = (joy->axes[2])*(0.5);
		joy_cmd_lateral = (joy->axes[3])*(-0.35);
		joy_cmd_steering = (joy->axes[0])*(0.7);
		joy_cmd_elevation = (joy->axes[1])*(0.95);

		vh1_->speed[0] = joy_cmd_forward;
		vh1_->speed[1] = joy_cmd_lateral;
		vh1_->speed[2] = joy_cmd_steering;

		joy_cmd_hit = joy->axes[4]*(-500.0);
		joy_cmd_spin = joy->axes[5]*(-500.0);

		if(joy->buttons[0] == 1)
		{
			if(launcher_lower_speed>25000 && joy_cmd_hit>0) 
			{	
				joy_cmd_hit = 0;
			}
			else if(launcher_lower_speed<1000 && joy_cmd_hit<0)
			{
				joy_cmd_hit = 0;
			}

			if(launcher_upper_speed>25000 && joy_cmd_hit>0) 
			{	
				joy_cmd_hit = 0;
			}
			else if(launcher_upper_speed<1000 && joy_cmd_hit<0)
			{
				joy_cmd_hit = 0;
			}

			if((launcher_upper_speed - launcher_lower_speed>=12000) && joy_cmd_spin>0) 
			{
				joy_cmd_spin = 0;
			}	
			else if((launcher_lower_speed - launcher_upper_speed>=12000) && joy_cmd_spin<0) 
			{
				joy_cmd_spin = 0;
			}		

			launcher_lower_speed += joy_cmd_hit - joy_cmd_spin;
			launcher_upper_speed += joy_cmd_hit + joy_cmd_spin;
		}
		else
		{
			launcher_lower_speed = 0;
			launcher_upper_speed = 0;
		}

		// ball feeder on
		feeder.data = false;
		if(joy->buttons[1] == 1)
		{
			// ball feeding On
			//vh1_->duty[7] = 0.95;
			feeder.data = true;
			vh1_->odroid_gpio_FEEDER.publish(feeder);
			vh1_->led[2] = 1;
		}
		else if(joy->buttons[1] == 0)
		{
			// ball feeding Off
			//vh1_->duty[7] = 0.;
			feeder.data = false;
			vh1_->odroid_gpio_FEEDER.publish(feeder);
			vh1_->led[2] = 0;
		}

		// launcher speed
		//vh2_->speed[0] = launcher_lower_speed;
		//vh2_->speed[1] = -launcher_upper_speed;
		vh1_->speed[5] = launcher_lower_speed;
		vh1_->speed[6] = -launcher_upper_speed;
		
		// Elevation
		vh1_->duty[4] = joy_cmd_elevation;
		
		ROS_INFO("lc_low:%.3f, lc_up:%.3f, ang:%.3f", launcher_lower_speed, launcher_upper_speed, this->launcher_incline_angle);
	}

	if(joy_dev_name == "/dev/input/js1")	
	{
		//ROS_INFO("joy device: %s", joy_dev_name.c_str());
		//ROS_INFO("%d, %d, %d, %d, %d, %d", joy->buttons[0], joy->buttons[1], joy->buttons[2], 
		//								   joy->buttons[3], joy->buttons[4], joy->buttons[5]);

		// Movement
		if(joy->buttons[0]==1)
		{
			// go forward
			joy_cmd_forward = -0.5;
		}
		else if(joy->buttons[4]==1)
		{
			// go backward
			joy_cmd_forward = 0.5;
		}

		if(joy->buttons[1]==1)
		{
			joy_cmd_steering = 0.7;
		}
		else if(joy->buttons[2]==1)
		{
			joy_cmd_steering = -0.7;
		}
		
		// Ball Launcher Power
		if(joy->buttons[3]==1 && joy->buttons[0]==1)
		{
			// Ball Launcher Power Up
			joy_cmd_hit += 1000;
			joy_cmd_forward = 0;
		}
		else if(joy->buttons[3]==1 && joy->buttons[4]==1)
		{
			// Ball Launcher Power Down
			joy_cmd_hit -= 1000;
			joy_cmd_forward = 0;
		}
		if(joy->buttons[0]==1 && joy->buttons[4]==1)
		{
			// Ball Launcher Power Off
			launcher_lower_speed = 0;
			launcher_upper_speed = 0;
			joy_cmd_forward = 0;
		}

		// Ball Launcher Spin
		if(joy->buttons[3]==1 && joy->buttons[1]==1)
		{
			// Ball Launcher Spin - Drive
			joy_cmd_spin -= 200;
			joy_cmd_steering = 0;
		}
		else if(joy->buttons[3]==1 && joy->buttons[2]==1)
		{
			// Ball Launcher Spin - Slice
			joy_cmd_spin += 200;
			joy_cmd_steering = 0;
		}

		if(launcher_lower_speed>=25000 && joy_cmd_hit>0) 
		{	
			launcher_lower_speed = 25000;
			joy_cmd_hit = 0;
		}
		else if(launcher_lower_speed<=0 && joy_cmd_hit<0)
		{
			launcher_lower_speed = 0;
			joy_cmd_hit = 0;
		}

		if(launcher_upper_speed>=25000 && joy_cmd_hit>0) 
		{	
			launcher_upper_speed = 25000;
			joy_cmd_hit = 0;
		}
		else if(launcher_upper_speed<=1000 && joy_cmd_hit<0)
		{
			launcher_upper_speed = 1000;
			joy_cmd_hit = 0;
		}

		if((launcher_upper_speed - launcher_lower_speed>=12000) && joy_cmd_spin>0) 
		{
			joy_cmd_spin = 0;
		}	
		else if((launcher_lower_speed - launcher_upper_speed>=12000) && joy_cmd_spin<0) 
		{
			joy_cmd_spin = 0;
		}

		launcher_lower_speed += joy_cmd_hit - joy_cmd_spin;
		launcher_upper_speed += joy_cmd_hit + joy_cmd_spin;

		// Elevation
		if(joy->buttons[5]==1 && joy->buttons[0]==1)
		{
			// elevation up
			joy_cmd_elevation = 0.95;
			joy_cmd_forward = 0;
		}
		else if(joy->buttons[5]==1 && joy->buttons[4]==1)
		{
			// elevation down
			joy_cmd_elevation = -0.95;
			joy_cmd_forward = 0;
		}

		// Ball feeder on/off
		if(joy->buttons[3]==1 && joy->buttons[5]==1)
		{
			if(feeder.data == true)
			{
				// feeder off
				feeder.data = false;
				vh1_->odroid_gpio_FEEDER.publish(feeder);
				vh1_->led[2] = 0;
			}
			else
			{
				// feeder on
				feeder.data = true;
				vh1_->odroid_gpio_FEEDER.publish(feeder);
				vh1_->led[2] = 1;
			}
		}

		// Program 1 - best drive ball
		if(joy->buttons[5]==1 && joy->buttons[2]==1)
		{
			// prog1 - left&right on
			launcher_lower_speed = 7800;
			launcher_upper_speed = 12200;
		} 

		// Program 2 - left/right sway
		if(joy->buttons[5]==1 && joy->buttons[1]==1)
		{
			// prog1 - left&right on
			if(vh1_->prog_flag[0] == 0) 
			{
				vh1_->prog_flag[0] = 1;
			} 
			else if(vh1_->prog_flag[0] == 1)
			{
				vh1_->prog_flag[0] = 0;
			}
		} 

		// Mecanum Speed
		vh1_->speed[0] = joy_cmd_forward;
		vh1_->speed[1] = 0;
		vh1_->speed[2] = joy_cmd_steering;

		vh1_->duty[4] = -joy_cmd_elevation;

		// Launcher Speed
		vh1_->speed[5] = launcher_lower_speed;
		vh1_->speed[6] = -launcher_upper_speed;

		ROS_INFO("lc_low:%.3f, lc_up:%.3f, ang:%.3f", launcher_lower_speed, launcher_upper_speed, this->launcher_incline_angle);
	}
}

void TeleopInput::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	//ROS_INFO("Imu Seq: [%d]", msg->header.seq);
	//ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  double roll, pitch, yaw;
  tf::Quaternion quat(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  //ROS_INFO("Roll: [%f], Pitch: [%f], Yaw: [%f]", roll*RAD2DEG, pitch*RAD2DEG, yaw*RAD2DEG);
	this->launcher_incline_angle = -pitch*RAD2DEG;
}

void TeleopVesc::LEDToggleRED(int flag)
{
	static ros::Time time_prev, time_diff;
	int sec = 0;

	sec = (ros::Time::now() - time_prev).toSec();

	if(sec >= 1 && flag==1)
	{
		if(this->led_R.data==false) 
		{
			this->led_R.data = true;
			this->odroid_gpio_LED_R.publish(led_R);
		}
		else
		{
			this->led_R.data = false;
			this->odroid_gpio_LED_R.publish(led_R);
		}
		time_prev = ros::Time::now();
	}
}

void TeleopVesc::LEDToggleYellow(int flag)
{
	static ros::Time time_prev, time_diff;
	int sec = 0;

	sec = (ros::Time::now() - time_prev).toSec();

	if(sec >= 1 && flag==1)
	{
		if(this->led_Y.data==false) 
		{
			this->led_Y.data = true;
			this->odroid_gpio_LED_Y.publish(led_Y);
			this->speed[2] = 0.5;
		}
		else
		{
			this->led_Y.data = false;
			this->odroid_gpio_LED_Y.publish(led_Y);
			this->speed[2] = -0.5;
		}
		time_prev = ros::Time::now();
	}
}

void TeleopVesc::LEDToggleGreen(int flag)
{
	static ros::Time time_prev, time_diff;
	int sec = 0;

	sec = (ros::Time::now() - time_prev).toSec();

	if(sec >= 1 && flag==1)
	{
		if(this->led_G.data==false) 
		{
			this->led_G.data = true;
			this->odroid_gpio_LED_G.publish(led_G);
		}
		else
		{
			this->led_G.data = false;
			this->odroid_gpio_LED_G.publish(led_G);
		}
		time_prev = ros::Time::now();
	}
}

void TeleopVesc::customsCallback(const vesc_msgs::VescGetCustomApp::ConstPtr& custom_rx_msg)
{
#ifdef PRINT_SENSOR_CUSTOMS
	ROS_INFO("------------------------------------------");
	ROS_INFO("header:%6.4f", custom_rx_msg->header.stamp.toSec());
	//ROS_INFO("voltage input:%.2f V", custom_rx_msg->voltage_input);
	//ROS_INFO("temperature pcb:%.2f C", custom_rx_msg->temperature_pcb);
	//ROS_INFO("current motor:%.2f A", custom_rx_msg->current_motor);
	//ROS_INFO("current input:%.2f A", custom_rx_msg->current_input);
	//ROS_INFO("erpm:%.2f", custom_rx_msg->speed);
	//ROS_INFO("duty:%.2f", custom_rx_msg->duty_cycle);
	//ROS_INFO("amp_hours:%.2f", custom_rx_msg->charge_drawn);
	//ROS_INFO("amp_hours_charged:%.2f", custom_rx_msg->charge_regen);
	//ROS_INFO("watt_hours:%.2f", custom_rx_msg->energy_drawn);	
	//ROS_INFO("watt_hours_charged:%.2f", custom_rx_msg->energy_regen);
	//ROS_INFO("tachometer:%.2f", custom_rx_msg->displacement);
	//ROS_INFO("tachometer_abs:%.2f", custom_rx_msg->distance_traveled);
	ROS_INFO("fault code:%d", custom_rx_msg->fault_code);
	//ROS_INFO("pid_pos_now:%.2f", custom_rx_msg->pid_pos_now);
	ROS_INFO("rps_0:%.2f, current_0:%.2f, duty_0:%.2f", custom_rx_msg->enc_rps[0], custom_rx_msg->current[0], custom_rx_msg->duty[0]);
	ROS_INFO("app status code:%d", custom_rx_msg->app_status_code);
#endif

	this->app_status_code = custom_rx_msg->app_status_code;
	
	for(int i=0; i<this->NO_VESC; i++) {
		this->controller_id[i] = custom_rx_msg->can_id[i];
		this->rps[i] = custom_rx_msg->enc_rps[i];
		//rad[i] = custom_rx_msg->enc_rad[i];
		this->current_status[i] = custom_rx_msg->current[i];
		this->duty_status[i] = custom_rx_msg->duty[i];
        this->custom_status[i] = custom_rx_msg->custom_status[i];
    }

	this->last_app_status_code = this->app_status_code;
	for(int i=0; i<this->NO_VESC; i++) this->last_custom_status[i] = this->custom_status[i];
}

void TeleopVesc::stateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state_msg)
{
#ifdef PRINT_SENSOR_CORE
	ROS_INFO("------------------------------------------");
	ROS_INFO("header:%6.4f", state_msg->header.stamp.toSec());
	ROS_INFO("voltage input:%.2f V", state_msg->state.voltage_input);
	ROS_INFO("temperature pcb:%.2f C", state_msg->state.temperature_pcb);
	ROS_INFO("current motor:%.2f A", state_msg->state.current_motor);
	ROS_INFO("current input:%.2f A", state_msg->state.current_input);
	ROS_INFO("erpm:%.2f", state_msg->state.speed);
	ROS_INFO("duty:%.2f", state_msg->state.duty_cycle);
	ROS_INFO("amp_hours:%.2f", state_msg->state.charge_drawn);
	ROS_INFO("amp_hours_charged:%.2f", state_msg->state.charge_regen);
	ROS_INFO("watt_hours:%.2f", state_msg->state.energy_drawn);
	ROS_INFO("watt_hours_charged:%.2f", state_msg->state.energy_regen);
	ROS_INFO("tachometer:%.2f", state_msg->state.displacement);
	ROS_INFO("tachometer_abs:%.2f", state_msg->state.distance_traveled);
	ROS_INFO("fault code:%d", state_msg->state.fault_code);
	ROS_INFO("pid_pos_now:%.2f", state_msg->pid_pos_now);
	ROS_INFO("controller_id:%d", custom_rx_msg->controller_id);
#endif
}

void TeleopVesc::setCmdMsg(double data, int send_can, int can_id)
{
	cmd_msg.data = data;
	cmd_msg.send_can = send_can;
	cmd_msg.can_id = can_id;
}

void TeleopVesc::setCustomMsg(int can_id, int send_can, int cmd_type, double data)
{
	//
	custom_tx_msg.id_set.push_back(can_id);
	custom_tx_msg.can_forward_set.push_back(send_can);
	custom_tx_msg.comm_set.push_back(cmd_type);
	custom_tx_msg.value_set.push_back(data);
}

void TeleopVesc::requestCustoms()
{
	std_msgs::Bool msg;
	msg.data = true;
	vesc_cmd_get_customs.publish(msg);
}

void TeleopVesc::setCustomOut()
{
	int num_of_id = 0;
	int can_forw = 0;
	int no_vesc = 0;

	no_vesc = this->NO_VESC;
	
	// Clear Custom Message
	custom_tx_msg.id_set.clear();
	custom_tx_msg.can_forward_set.clear();
	custom_tx_msg.comm_set.clear();
	custom_tx_msg.value_set.clear();

	// Custom Command
	for(int i=0; i<no_vesc; i++) {
		if(i==0) can_forw = CAN_FORWARD_OFF;
		else     can_forw = CAN_FORWARD_ON;
		setCustomMsg(i, can_forw, custom_cmd_type[i], custom_cmd_value[i]);
		num_of_id++;
	}
	custom_tx_msg.num_of_id = num_of_id;
	custom_tx_msg.data_bytes = 2 + 6*num_of_id;
	vesc_cmd_set_customs.publish(custom_tx_msg);
}

void TeleopVesc::setCurrentOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		if(this->port_name=="/dev/ttyVESC3")
		{
			// current
			for(int i=0; i<=1; i++) {
				if(i==0) can_forw = CAN_FORWARD_OFF;
				else     can_forw = CAN_FORWARD_ON;
				setCmdMsg(this->current[i], can_forw, i);
				this->vesc_cmd_current.publish(cmd_msg);
			}
		}
	}
}

void TeleopVesc::setBrakeOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		// current
		for(int i=0; i<=1; i++) {
			if(i==0) can_forw = CAN_FORWARD_OFF;
			else     can_forw = CAN_FORWARD_ON;
			setCmdMsg(brake[i], can_forw, i);
			vesc_cmd_brake.publish(cmd_msg);
		}
	}
}

void TeleopVesc::setDutyCycleOut()
{
	int can_forw = 0;

	if(this->enable.data)
	{
		if(this->port_name=="/dev/ttyVESC3")
		{
			// duty
			//for(int i=0; i<=3; i++) {
			for(int i=0; i<=4; i++) {	// including elevation
				if(i==0) can_forw = CAN_FORWARD_OFF;
				else     can_forw = CAN_FORWARD_ON;
				setCmdMsg(this->duty[i], can_forw, i);
				this->vesc_cmd_duty.publish(cmd_msg);
			}

			/*
			// ball feeder
			setCmdMsg(this->duty[7], CAN_FORWARD_ON, 7);
			this->vesc_cmd_duty.publish(cmd_msg);
			*/
		}
	}
}

void TeleopVesc::setSpeedOut()
{
	int can_forw = 0;

	if(this->enable.data)
	{
		if(this->port_name=="/dev/ttyVESC3")
		//if(this->port_name=="/dev/ttyVESC3")
		{
			// speed
			for(int i=5; i<=6; i++) {
				if(i==0) can_forw = CAN_FORWARD_OFF;
				else     can_forw = CAN_FORWARD_ON;
				setCmdMsg(this->speed[i], can_forw, i);
				this->vesc_cmd_speed.publish(cmd_msg);
			}
		}
	}
}

void TeleopVesc::setPositionOut()
{
	int can_forw = 0;

	if(enable.data)
	{
		// position
		for(int i=0; i<=1; i++) {
			if(i==0) can_forw = CAN_FORWARD_OFF;
			else     can_forw = CAN_FORWARD_ON;
			setCmdMsg(enc_deg[i], can_forw, i);
			vesc_cmd_position.publish(cmd_msg);
		}
	}
}

double duty_limit(double input_duty)
{
	double duty_limit = 0.95;
	double output_duty;

	if(input_duty>=duty_limit)	output_duty = duty_limit;
	else if(input_duty<=-duty_limit)	output_duty = -duty_limit;
	else output_duty = input_duty;

	return output_duty;
}

void mecanum_robot_jacobian(double vx, double vy, double wz, double *duty1, double *duty2, double *duty3, double *duty4)
{
	// robot parameter
	static double radius_wheel = 63.5/1000.;
	static double alpha = 0.325;	// alpha = L+l
	static double VEL2DUTY = 0.06;//0.02;
	static int 	  MOTOR_DIR[4] = {-1, -1, 1, 1};
	double u1, u2, u3, u4;
	
	u1 = MOTOR_DIR[0]*(1.0*vx - 1.0*vy - alpha*wz)/radius_wheel;
	u2 = MOTOR_DIR[1]*(1.0*vx + 1.0*vy - alpha*wz)/radius_wheel;
	u3 = MOTOR_DIR[2]*(1.0*vx - 1.0*vy + alpha*wz)/radius_wheel;
	u4 = MOTOR_DIR[3]*(1.0*vx + 1.0*vy + alpha*wz)/radius_wheel;

	*duty1 = duty_limit(u1*VEL2DUTY);
	*duty2 = duty_limit(u2*VEL2DUTY);
	*duty3 = duty_limit(u3*VEL2DUTY);
	*duty4 = duty_limit(u4*VEL2DUTY);
}

/*
 * Main Function
 * 
 */
int main(int argc, char **argv)            
{
  ros::init(argc, argv, "vesc_control_node");

  // loop freq
  int rate_hz = 100;	//hz

  // TeleopVesc Class
  TeleopVesc *teleop_vesc1 = new TeleopVesc(8, "/dev/ttyVESC3"); 

  // TeleopInput Class
  TeleopInput tele_input(teleop_vesc1, NULL, NULL);

  // Velocity Profile
  float amax = 0.5;
  static float vout_x, vout_y, vout_z;
  TrapezoidalVelProfile v_prof_x(amax, 1./rate_hz);
  TrapezoidalVelProfile v_prof_y(amax, 1./rate_hz);
  TrapezoidalVelProfile v_prof_z(4.*amax, 1./rate_hz);

  // ROS Loop
  int cnt_lp = 0;
  ros::Rate loop_rate(rate_hz); //Hz
  ROS_INFO("Start Tele-operation");
  teleop_vesc1->enable.data = true;
  //teleop_vesc2->enable.data = true;

  teleop_vesc1->startTime = ros::Time::now();
  //teleop_vesc2->startTime = ros::Time::now();

  while (ros::ok())
  { 
		// read encoder data.
		//teleop_vesc->requestCustoms();
		//ROS_INFO("rps_0:%.2f(dps_0:%.2f), rad_0:%.2f", teleop_vesc->rps[0], teleop_vesc->rps[0]*RPS2DPS, teleop_vesc->rad[0]);
		//ROS_INFO("rps_1:%.2f(dps_1:%.2f), rad_1:%.2f", teleop_vesc->rps[1], teleop_vesc->rps[1]*RPS2DPS, teleop_vesc->rad[1]);
		//ROS_INFO("rps_2:%.2f(dps_2:%.2f), rad_2:%.2f", teleop_vesc->rps[2], teleop_vesc->rps[2]*RPS2DPS, teleop_vesc->rad[2]);
		//ROS_INFO("rps_3:%.2f(dps_3:%.2f), rad_3:%.2f", teleop_vesc->rps[3], teleop_vesc->rps[3]*RPS2DPS, teleop_vesc->rad[3]);

		// current example (A)
		//teleop_vesc->current[0] = 4.0;
		//teleop_vesc->current[1] = 4.0;
		// teleop_vesc->current[2] = -1.0;
		// teleop_vesc->current[3] = 1.0;
		//teleop_vesc->setCurrentOut();

		// // brake example (A)
		// teleop_vesc->brake[0] = 10.0;
		// teleop_vesc->brake[1] = 10.0;
		// teleop_vesc->brake[2] = 5.0;
		// teleop_vesc->brake[3] = 8.0;
		// teleop_vesc->setBrakeOut();

		// // speed example (erpm = rev/min*polepair, polepair=Encoder Ratio@vesc_tool)
		//teleop_vesc->speed[0] = -20000.;//-(15000.0 - 5000.0);
		//teleop_vesc->speed[1] = 5000.;//(15000.0 + 5000.0);
		// teleop_vesc->speed[2] = -1000.0;
		// teleop_vesc->speed[3] = 5000.0;
		//teleop_vesc->setSpeedOut();

		// // // duty example (0.005~0.95)
		v_prof_x.GenProfile(teleop_vesc1->speed[0], &vout_x);
		v_prof_y.GenProfile(teleop_vesc1->speed[1], &vout_y);
		v_prof_z.GenProfile(teleop_vesc1->speed[2], &vout_z);
		mecanum_robot_jacobian(vout_x, vout_y, vout_z,
					  &(teleop_vesc1->duty[0]), &(teleop_vesc1->duty[1]), &(teleop_vesc1->duty[2]), &(teleop_vesc1->duty[3]));
		//teleop_vesc->duty[0] = 0.1;
		//teleop_vesc->duty[1] = 0.1;
		//teleop_vesc->duty[2] = 0.1;
		//teleop_vesc->duty[3] = 0.1;
		teleop_vesc1->setDutyCycleOut();

		//teleop_vesc1->custom_cmd_type[0] = COMM_SET_DPS;
		//teleop_vesc1->custom_cmd_value[0] = 100.;
		//teleop_vesc1->custom_cmd_type[1] = COMM_SET_DPS;
		//teleop_vesc1->custom_cmd_value[1] = 100.;
		//teleop_vesc1->custom_cmd_type[2] = COMM_SET_DPS;
		//teleop_vesc1->custom_cmd_value[2] = 100.;
		//teleop_vesc1->custom_cmd_type[3] = COMM_SET_DPS;
		//teleop_vesc1->custom_cmd_value[3] = 100.;
		//teleop_vesc1->setCustomOut();

		//teleop_vesc->custom_cmd_value[0] = teleop_vesc->dps[0]*amplitude*2*M_PI*freq*cos(2*M_PI*freq*(ros::Time::now() - teleop_vesc->startTime).toSec());
		//teleop_vesc->custom_cmd_type[1] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[1] = 1000.;
		//teleop_vesc->custom_cmd_type[2] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[2] = -1000.;


		//ROS_INFO("duty_0:%.2f, duty_1:%.2f, duty_2:%.2f, duty_3:%.2f", teleop_vesc->duty[0], teleop_vesc->duty[1], teleop_vesc->duty[2], teleop_vesc->duty[3]);

		// // position example (0~360 deg)
		//teleop_vesc->position[0] = 0.;
		// teleop_vesc->position[1] = 15.;
		// teleop_vesc->position[2] = 270.;
		// teleop_vesc->position[3] = -4;
		//teleop_vesc->setPositionOut();

		//teleop_vesc2->setSpeedOut();
		teleop_vesc1->setSpeedOut();

		// Custom example
		//freq = teleop_vesc->speed[0];
		//teleop_vesc->custom_cmd_type[0] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[0] = teleop_vesc->dps[0]*amplitude*2*M_PI*freq*cos(2*M_PI*freq*(ros::Time::now() - teleop_vesc->startTime).toSec());
		//teleop_vesc->custom_cmd_type[1] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[1] = 1000.;
		//teleop_vesc->custom_cmd_type[2] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[2] = -1000.;
		//teleop_vesc->custom_cmd_type[3] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[3] = 1000.;
		// teleop_vesc->custom_cmd_type[1] = COMM_SET_DPS;
		//teleop_vesc->custom_cmd_value[1] = 0.;
		//teleop_vesc->setCustomOut();

		// Error : Toggle RED LED
		//teleop_vesc1->LEDToggleRED(teleop_vesc1->led[0]);

		// program 1
		teleop_vesc1->LEDToggleYellow(teleop_vesc1->prog_flag[0]);

		// Feeder On : Toggle Green LED
		teleop_vesc1->LEDToggleGreen(teleop_vesc1->led[2]);

		// once in 1sec
		std_msgs::Bool led_data;
		if(cnt_lp>=rate_hz) 
		{
			// Power On : Red On
			led_data.data = true;	// false is ON, only in case of red
			teleop_vesc1->odroid_gpio_LED_Y.publish(led_data);

			//
			//ROS_INFO("vout_x:%.2f(%.2f), vout_y:%.2f(%.2f), vout_z:%.2f(%.2f)", 
			//	vout_x,teleop_vesc1->speed[0], vout_y,teleop_vesc1->speed[1], vout_z,teleop_vesc1->speed[2]);

			cnt_lp = 0;
		}

		ros::spinOnce();
		loop_rate.sleep();
		cnt_lp++;
  }

  return 0;
}
