#include "stdafx.h"
#include "Comm_data_motor3.h"
#include "Comm_data_star.h" //红外标签定位模块
//#include "MotorTest.h"

#include "Plan_Path_VFH.h"

#define COS30f  0.86602540
#define SIN30f  0.5
//float R  =    220; // robot radius: 18cm
#define PIf 3.1415926
//float Wheel_radius = 50.00; ////mm
extern Cstar StarGazer;
extern CPlan_Path_VFH plan;

float dic1=0.0,dic2=0.0,dic3=0.0;//xhy:跳变检测
float overx1=0.0,overx2=0.0,overx3=0.0;//xhy:出界检测
float overy1=0.0,overy2=0.0,overy3=0.0;//xhy:出界检测
float overa1=0.0,overa2=0.0,overa3=0.0;//xhy:出界检测
float overx12=0.0,overx22=0.0,overx32=0.0;//xhy:出界检测
float overy12=0.0,overy22=0.0,overy32=0.0;//xhy:出界检测
float overa12=0.0,overa22=0.0,overa32=0.0;//xhy:出界检测

int starmust=0;//强制校准
int starnew=0;//同步频率

float speedslowchang=0.0;
float wlowchang=0.0;

struct StarMark
{
	int markID;
	float mark_angle;
	float mark_x;
	float mark_y;
};

extern struct StarMark MARK[100];

extern long int whichturn;

CMotor::CMotor(void)
{
	m_baudrate=115200;
	motor_str[0]=':';
	motor_str[1]='L';
	motor_str[2]='F';
	motor_str[3]='0';
	motor_str[4]='0';
	motor_str[5]='0';
	motor_str[6]='R';
	motor_str[7]='F';
	motor_str[8]='0';
	motor_str[9]='0';
	motor_str[10]='0';
	motor_str[11]='Z';
	motor_str[12]='F';
	motor_str[13]='0';
	motor_str[14]='0';
	motor_str[15]='0';
	motor_str[16]='G';
	motor_str[17]='.';

	Go_status = 0;
	Go_status_old = 0;
	m_lastChar = 0;
	m_ParHeader = '*'; 
	m_ParEnd = '#';
	Time_start='T';
	ldata_star = 'L';
	rdata_star = 'R';
	zdata_star = 'Z';

//	m_recvbuf = new char[100]; //new动态分配内存,数据包储存
	m_databuf = new char[100]; //数据存储
	m_nRecvindex = 0;       //数据写入位，m_recvbuf
	m_nFrameLen = 0;        //存储数据包的长度
	m_bFrameStart = false;  //数据包接收开始标志位，TRUE为已开始接收，FLASE为没有开始
	m_right=false;
}


CMotor::~CMotor(void)
{
	delete m_databuf;
}

void CMotor::Parse(BYTE inData)
{
	
	if (m_bFrameStart == false) //若没有开始接收数据，则需要检测是否有数据包头出现，数据包接收开始标志位，TRUE为已开始接收，FLASE为没有开始
	{
		//判断是否为包头
		if (inData == Time_start && m_lastChar == m_ParHeader)//数据包头出现则开始接收数据
		{
			m_bFrameStart = true;
			m_recvbuf[0]='*';
			m_recvbuf[1]='T';
			//写入包头，把包头写入到数据包存储
			m_nRecvindex = 1;//数据写入位
		}
	}
	else//若已经开始接收数据
	{
			//解析中
			m_nRecvindex++;//数据写入位加1
			m_recvbuf[m_nRecvindex] = inData;
			//寻找包尾
			if (m_nRecvindex%25 == 0)//若数据和上一个接收到的字符均为数据包结束格式
			{
				if (inData == m_ParEnd)//数据长度是否正确 共19个字节数据
				{
					m_nFrameLen = m_nRecvindex + 1;//存储数据包的长度等于数据写入位加1
					m_ParseFrame();//对接收的数据包进行解析
					encoder_l = lift_num;
					encoder_r = right_num;
					encoder_z = zhong_num;
					timer = motor_timer;
					memset(m_recvbuf,0x00,sizeof(m_recvbuf));
				}
				
				m_bFrameStart = false;
				m_nRecvindex = 0;
				m_nFrameLen = 0;
				
			}
			
	}
	m_lastChar = inData;//把上一个接收到的字符更改成inData
	//memset(m_recvbuf,0x00,sizeof(m_recvbuf));
}

void CMotor::m_ParseFrame(void)//串口数据解析
{
	for (int i = 0;i<m_nFrameLen-1;i++)
	{
		if (m_recvbuf[i]==Time_start&&m_recvbuf[i+5]==';')
		{
		    motor_timer=m_recvbuf[i+1];
			motor_timer=(motor_timer<<8)+m_recvbuf[i+2];
			motor_timer=(motor_timer<<8)+m_recvbuf[i+3];
			motor_timer=(motor_timer<<8)+m_recvbuf[i+4];
//			fprintf(file,"\t\t recdatatime:		%d    \n",motor_timer);
		}
		if (m_recvbuf[i]==ldata_star&&m_recvbuf[i+5]==';')
		{
			lift_num=m_recvbuf[i+1]; 
			lift_num=(lift_num<<8)+m_recvbuf[i+2];
			lift_num=(lift_num<<8)+m_recvbuf[i+3];
			lift_num=(lift_num<<8)+m_recvbuf[i+4];
			//lift_num = lift_num*(-1);
		}
		if (m_recvbuf[i]==rdata_star&&m_recvbuf[i+5]==';')
		{
			right_num=m_recvbuf[i+1];
			right_num=(right_num<<8)+m_recvbuf[i+2];
			right_num=(right_num<<8)+m_recvbuf[i+3];
			right_num=(right_num<<8)+m_recvbuf[i+4];
		//	right_num = right_num*(-1);
		}
		if (m_recvbuf[i]==zdata_star&&m_recvbuf[i+5]==';')
		{
			zhong_num=m_recvbuf[i+1];
			zhong_num=(zhong_num<<8)+m_recvbuf[i+2];
			zhong_num=(zhong_num<<8)+m_recvbuf[i+3];
			zhong_num=(zhong_num<<8)+m_recvbuf[i+4];
		}

//		fprintf(file,"\t\t recdataspeed_l:		%d    ",lift_num);
//		fprintf(file,"\t\t recdataspeed_r:		%d    \n",right_num);

		if (m_recvbuf[i]==0x0d&&m_recvbuf[i+1]==0x0a)
		{
			return;
		}
	}
	//out=fopen("re.txt","a+");
	//fprintf(out,"\n motor_timer:%d...",motor_timer);
	//fprintf(out,"lift_num:%d...",lift_num);
	//fprintf(out,"right_num:%d...\n",right_num);
	//fclose(out);
}

extern FILE* file;

//机器人运动函数：直接通过给左右轮轮速控制机器人运动
bool CMotor::gomotor(int Lspeed, int Rspeed,int Zspeed)//Lspeed +为正转-为反转 绝对值为速度 单位cm/s
{
	motor_str[0]=':';
	motor_str[1]='L';
	motor_str[2]='F';
	motor_str[3]='0';
	motor_str[4]='0';
	motor_str[5]='0';
	motor_str[6]='R';
	motor_str[7]='F';
	motor_str[8]='0';
	motor_str[9]='0';
	motor_str[10]='0';
	motor_str[11]='Z';
	motor_str[12]='F';
	motor_str[13]='0';
	motor_str[14]='0';
	motor_str[15]='0';
	motor_str[16]='G';
	motor_str[17]='.';
	if (Lspeed<=0)
	{
		motor_str[2]='B';
	}
	else 
	{
		motor_str[2]='F';
	}
	if (Rspeed<=0)
	{
		motor_str[7]='B';
	}
	else 
	{
		motor_str[7]='F';
	}
	if (Zspeed<0)
	{
		motor_str[12] = 'B';
	} 
	else
	{
		motor_str[12] = 'F';
	}
	if (Lspeed>=0&&Rspeed>=0) Go_status=0;
	if (Lspeed>=0&&Rspeed<0) Go_status=1;
	if (Lspeed<0&&Rspeed>=0) Go_status=2;
	if (Lspeed<0&&Rspeed<0) Go_status=3;

	//if (Go_status_old!=Go_status) //本次状态与上次状态不同，则发送停止命令，返回false，需要再次发送命令达成目的
	//{	
	//	stop();
	//	Go_status_old=Go_status;
	//	Sleep(20);
	////	return false;
	//}
	motor_str[16]='G';
	Lspeed=abs(Lspeed);
	Rspeed=abs(Rspeed);
	Zspeed = abs(Zspeed);

	motor_str[3]=Lspeed/100+0x30;
	motor_str[4]=(Lspeed%100)/10+0x30;
	motor_str[5]=Lspeed%10+0x30;

	motor_str[8]=Rspeed/100+0x30;
	motor_str[9]=(Rspeed%100)/10+0x30;
	motor_str[10]=Rspeed%10+0x30;

	motor_str[13]=Zspeed/100+0x30;
	motor_str[14]=(Zspeed%100)/10+0x30;
	motor_str[15]=Zspeed%10+0x30;

	
	/*fprintf(file,"senddata:		%c",motor_str[0]);
	fprintf(file,"%c",motor_str[1]);
	fprintf(file,"%c",motor_str[2]);
	fprintf(file,"%c",motor_str[3]);
	fprintf(file,"%c",motor_str[4]);
	fprintf(file,"%c",motor_str[5]);
	fprintf(file,"%c",motor_str[6]);
	fprintf(file,"%c",motor_str[7]);
	fprintf(file,"%c",motor_str[8]);
	fprintf(file,"%c",motor_str[9]);
	fprintf(file,"%c",motor_str[10]);
	fprintf(file,"%c",motor_str[11]);
	fprintf(file,"%c",motor_str[12]);
	fprintf(file,"%c",motor_str[13]);
	fprintf(file,"%c\n",motor_str[14]);*/

	ComSend(motor_str,18);
//	ComSend(":LF010RF010ZF000G.",18);
	memset(motor_str,0x00,sizeof(motor_str));
	return true;
}

bool CMotor::stop()
{
	ComSend(":LF000RF000ZF000G.",18);
	return true;
}


// 弧度转换控制函数
//bool CMotor::Velocity_control(float linear_velocity, float angular_velocity)// linear_velocity 线速度 angular_velocity 角速度 
//{
//	//move_lsp=0;
//	//move_rsp=0;
//	
//	if (linear_velocity>0&&angular_velocity>0) Go_status=0;
//	if (linear_velocity>0&&angular_velocity<0) Go_status=1;
//	if (linear_velocity<0&&angular_velocity>0) Go_status=2;
//	if (linear_velocity<0&&angular_velocity<0) Go_status=3;
//	
////	if (Go_status_old!=Go_status) //本次状态与上次状态不同，则发送停止命令，返回false，需要再次发送命令达成目的
////	{	
////		move_rsp=0;
////		move_lsp=0;
////		gomotor(move_lsp,move_rsp);
////		Go_status_old=Go_status;
//////		return false;
////	}
//	if (fabs(linear_velocity)<=0.05&&fabs(angular_velocity)<=0.05)
//	{
//		move_lsp=0;
//		move_rsp=0;
//		move_zsp=0;
//	}
//	else if (fabs(angular_velocity)<0.05)
//	{
//		move_lsp=linear_velocity;
//		move_rsp=linear_velocity;
//	}
//	else
//	{
//		move_rsp=(int)(2*linear_velocity+Drobot_wheel_spacing/10*angular_velocity)/2;//顺时针旋转为负，逆时针为正
//		move_lsp=(int)(2*linear_velocity-Drobot_wheel_spacing/10*angular_velocity)/2;	
//	}
//	if (move_lsp>19)
//	{
//		move_rsp -= move_lsp-19;
//		move_lsp = 19;
//	}
//	if (move_rsp>19)
//	{
//		move_lsp -= move_rsp-19;
//		move_rsp = 19;
//	}
//	/*gomotor(move_lsp*0.3,move_rsp*0.3);
//	Sleep(100);
//	gomotor(move_lsp*0.6,move_rsp*0.6);
//	Sleep(100);*/
//	if (move_lsp<0 && move_rsp<0)
//	{
//		if (move_lsp< -5)
//		{
//			move_lsp = -5;
//		}
//		if (move_rsp < -5)
//		{
//			move_rsp = -5;
//		}
//	}
//	gomotor(move_lsp,move_rsp);
/////	speed_l = move_lsp;
////	speed_r = move_rsp;
//	return true;
//	
//}

//void CMotor::VectorMove(double inAngle, float inLV, float inPSpeed) //左右前后移动控制，三个参数分别代表机器人坐标系与全局坐标系在水平方向的夹角，机器人运动的线速度，机器人运动的角速度
//{
////	int motor[3];	//后 左前 右前，
////	motor[2] =-(int(sin((double(m_CalAngle((int)-inAngle,+180)))*3.14/180)*inLV)-inPSpeed);	
////	motor[0] =-(int(sin((double(m_CalAngle((int)-inAngle,60)))*3.14/180)*inLV)-inPSpeed);	
////	motor[1] =-(int(sin((double(m_CalAngle((int)-inAngle,-60)))*3.14/180)*inLV)-inPSpeed);	////各轮线速度
//////	SetFourMotorsSpeed(motor[0],motor[1],motor[2],0);
////	gomotor(motor[1],motor[0],motor[2]);
//
//
//
//		//move_lsp=0;
//		//move_rsp=0;
//		
//		//if (linear_velocity>0&&angular_velocity>0) Go_status=0;
//		//if (linear_velocity>0&&angular_velocity<0) Go_status=1;
//		//if (linear_velocity<0&&angular_velocity>0) Go_status=2;
//		//if (linear_velocity<0&&angular_velocity<0) Go_status=3;
//		
//	//	if (Go_status_old!=Go_status) //本次状态与上次状态不同，则发送停止命令，返回false，需要再次发送命令达成目的
//	//	{	
//	//		move_rsp=0;
//	//		move_lsp=0;
//	//		gomotor(move_lsp,move_rsp);
//	//		Go_status_old=Go_status;
//	////		return false;
//	//	}
//		float linear_velocity = inLV;
//		float angular_velocity = inPSpeed;
//		if (fabs(linear_velocity)<=0.05&&fabs(angular_velocity)<=0.05)
//		{
//			move_lsp=0;
//			move_rsp=0;
//			move_zsp=0;
//		}
//		else if (fabs(angular_velocity)<0.05)
//		{
//			move_lsp=linear_velocity;
//			move_rsp=linear_velocity;
//		}
//		else
//		{
//			/*move_rsp=(int)(2*linear_velocity+Drobot_wheel_spacing/10*angular_velocity)/2;				
//			move_lsp=(int)(2*linear_velocity-Drobot_wheel_spacing/10*angular_velocity)/2;*/
//			/*if (angular_velocity > 0)
//			{*/
//				move_lsp = (int) linear_velocity;
//				move_rsp = (int) (linear_velocity + angular_velocity * 10);
//			/*}*/
//
//		/*	if (angular_velocity < 0)
//			{
//				move_lsp = (int) linear_velocity;
//				move_rsp = (int) (linear_velocity + angular_velocity * Drobot_wheel_spacing);
//			}*/
//		}
//		if (move_lsp > 20)
//		{
//			move_rsp -= move_lsp - 20;
//			move_lsp = 20;
//		}
//		if (move_rsp > 20)
//		{
//			move_lsp -= move_rsp - 20;
//			move_rsp = 20;
//		}
//
//		/*gomotor(move_lsp*0.3,move_rsp*0.3);
//		Sleep(100);
//		gomotor(move_lsp*0.6,move_rsp*0.6);
//		Sleep(100);*/
//		if (move_lsp<0 && move_rsp<0)
//		{
//			if (move_lsp< -5)
//			{
//				move_lsp = -5;
//			}
//			if (move_rsp < -5)
//			{
//				move_rsp = -5;
//			}
//		}
//		gomotor(move_lsp,move_rsp,(move_lsp-move_rsp));
//	///	speed_l = move_lsp;
//	//	speed_r = move_rsp;
//
//}

float Va = 0.0;
float Vb = 0.0;
float Vc = 0.0;

float Dtheat = 0.0;
float Dx = 0.0;
float Dy = 0.0;

float Vx = 0.0;
float Vy = 0.0;
float Vw = 0.0;
float dert = 0.0;
void CMotor::VectorMove(float inLV, float inPSpeed,float pianzhuan,float Robot_X,float Robot_Y) //左右前后移动控制，三个参数分别代表机器人坐标系与全局坐标系在水平方向的夹角，机器人运动的线速度，机器人运动的角速度
{
	FILE *alloutmxhy;
	alloutmxhy = fopen("alloutmotorxhy.txt","a+");
	fprintf(alloutmxhy,"1111==%f    %f     \n",inLV,inPSpeed);


	//////////////////////////////////xhy:变化插值
	if(speedslowchang-inLV>800||speedslowchang-inLV<-800)
	{
		inLV=inLV/3+2*speedslowchang/3;
	}
	speedslowchang=inLV;

	if(wlowchang-inPSpeed>1.0||wlowchang-inPSpeed<-1.0)
	{
		inPSpeed=inPSpeed/3+2*wlowchang/3;
	}
	wlowchang=inPSpeed;
	fprintf(alloutmxhy,"2222==%f    %f      %d  \n",inLV,inPSpeed,whichturn);
	fclose(alloutmxhy);
	////////////////////////////////
	plan.speed_line_pos=(int)inLV;
	//if(inLV>-0.001&&inLV<0.001&&(inPSpeed<-0.01||inPSpeed>0.01))
	//{
	//	inLV=200;
	//}
	float Radius_robot =     200.0;  // robot radius: 18cm
	float Wheel_radius = 50.00; ////mm
	double R_theta;
//	R_theta =atan(Robot_Y/Robot_X)-pianzhuan;
//	R_theta =pianzhuan;
	R_theta=0.0;

	Vx = inLV * cos(R_theta);
	Vy = inLV * sin(R_theta);	

	 move_lsp = int((sin(PIf/3 + R_theta) * (Vx) - cos(PIf/3 + R_theta) * (Vy) - Radius_robot * (inPSpeed))/Wheel_radius) ; 		
    move_zsp = int((-sin(R_theta) * (Vx) + cos(R_theta) * (Vy) -Radius_robot * (inPSpeed))/Wheel_radius) ;		
    move_rsp = int((-sin(PIf/3 - R_theta) * (Vx) - cos(PIf/3 - R_theta) * (Vy) - Radius_robot * (inPSpeed))/Wheel_radius); 





	//FILE *allout;
	//allout = fopen("allout3.txt","a+");
	//fprintf(allout,"%f    %f    %f    %f    %d  %d   %d  %f  %f  %f   \n",R_theta,Vx,Vy,inPSpeed, move_lsp, move_zsp, move_rsp,Robot_X,Robot_Y,pianzhuan);
	//fclose(allout);
	//if (move_lsp > 20)
	//{
	//	move_rsp -= move_lsp - 20;
	//	move_lsp = 20;
	//}
	//if (move_rsp > 20)
	//{
	//	move_lsp -= move_rsp - 20;
	//	move_rsp = 20;
	//}

	///*gomotor(move_lsp*0.3,move_rsp*0.3);
	//Sleep(100);
	//gomotor(move_lsp*0.6,move_rsp*0.6);
	//Sleep(100);*/
	//if (move_lsp<0 && move_rsp<0)
	//{
	//	if (move_lsp< -5)
	//	{
	//		move_lsp = -5;
	//	}
	//	if (move_rsp < -5)
	//	{
	//		move_rsp = -5;
	//	}
	//}
	gomotor(-move_lsp,-move_rsp,-move_zsp);//com345:负(如果方向全部相反一般是这里的问题)
///	speed_l = move_lsp;
//	speed_r = move_rsp;


}

int CMotor::m_CalAngle(int angle1, int angle2)//计算角度
{
	int ret;
	ret=angle2-angle1;
	if (ret>180)
		ret=-360+ret;
	if(ret<-180)
		ret=360+ret;
	return ret;
}

bool CMotor::open_com_motor(int CCommport)//打开电机串口
{
	return Create(CCommport);

}



//movexhy
void CMotor::RobotPositionCompute(float distancedif_l,float distancedif_r,float distancedif_z,robotinfo& Info_robot)/////从轮电机旋转的速度（电机坐标系）转换到全局坐标系中
{
	float dx = 0.0;
	float dy = 0.0;
	float dtheta = 0.0;
	float pointrox = Info_robot.pointrox;
	float pointroy = Info_robot.pointroy;
	double pian = Info_robot.pianzhuan;
	double pianzhuan = 0.0;

	FILE *allout;
	allout = fopen("allout2.txt","a+");
	FILE *alloutstar;
	alloutstar = fopen("alloustar.txt","a+");
	FILE *alloutsar;
	alloutsar = fopen("allotar.txt","a+");
	float  Radius_robot =     200.0;  // robot radius: 18cm
	float Wheel_radius = 50.00; ////mm
	double R_theta;
	/*Da = distancedif_l;
	Db = distancedif_r;
	Dc = distancedif_z;*/

	/* Da = distancedif_l;
	Db = distancedif_z;
	Dc = distancedif_r;*/
	/*float Da = distancedif_z;
	float Db = distancedif_r;
	float Dc = distancedif_l;*/
	/*float Da = distancedif_z*Wheel_radius;
	float Db = distancedif_r*Wheel_radius;
	float Dc = -distancedif_l*Wheel_radius;
	*/

	R_theta=Info_robot.pianzhuan;

	//////////////////////从motorToGlobal函数
	dx =(0.577350f*cos(R_theta) + 0.333333f*sin(R_theta))*(distancedif_l) -0.666667f * sin(R_theta)*(distancedif_z) + (-0.577350f*cos(R_theta) + 0.333333f*sin(R_theta))*(distancedif_r) ;
    dy = (0.577350f*sin(R_theta) - 0.333333f*cos(R_theta))*(distancedif_l)+0.666667f * cos(R_theta)*(distancedif_z) + (-0.577350f*sin(R_theta) - 0.333333f*cos(R_theta))*(distancedif_r)  ;
    dtheta = -( 1 / (float)(3*Radius_robot) ) * ( (distancedif_l) + (distancedif_z) +(distancedif_r) ) ;


	//double aoffset=atan2(dy,dx);
	//double alpha1=aoffset-pianzhuan;
	//alpha1=atan2(sin(alpha1),cos(alpha1));
	//double rho=sqrt(dx*dx+dy*dy);
	
//	fprintf(allout," 00000===%f     %f   %f  %f     %f    %f  %f  %f  %f   \n",pointrox,pointroy,pianzhuan,dx,dy,dtheta,distancedif_l,  distancedif_r,distancedif_z);

	pointrox += dx;//△x;
	pointroy += dy;//△y;
	pianzhuan += dtheta;
//	fprintf(allout," 11111===%f     %f   %f  %f     %f    %f  %f  %f  %f   \n",pointrox,pointroy,pianzhuan,dx,dy,dtheta,distancedif_l,  distancedif_r,distancedif_z);

	pianzhuan=atan2(sin(pianzhuan),cos(pianzhuan));

//	fprintf(allout," 2222===%f     %f   %f  %f     %f    %f  %f  %f  %f   \n",pointrox,pointroy,pianzhuan,dx,dy,dtheta,distancedif_l,  distancedif_r,distancedif_z);
//	fprintf(allout," 3333===%f    %f   %f     \n",StarGazer.starID,Info_robot.pointrox_stargazer,Info_robot.pointroy_stargazer);
		pian=pian+pianzhuan;
		if(pian<0)
		{
			pian+=2*PIf;
		}
		else if(pian>2*PIf)
		{
			pian-=2*PIf;
		}
	
	Info_robot.pointrox = pointrox;
	Info_robot.pointroy = pointroy;
	Info_robot.pianzhuan = pian;//xhy:改为自加，偏转为累计值

		fprintf(allout," 2222===%f     %f   %f  %f     %f    %f  %f  %f  %f   \n",pointrox,pointroy,pian*180/PIf,dx,dy,dtheta,distancedif_l,  distancedif_r,distancedif_z);
		/////////////////////////////////////

	float pointrox_stargazer1=0.0,pointroy_stargazer1=0.0;
	float pointrox_stargazer2=0.0,pointroy_stargazer2=0.0;
	float pointroA_stargazer1=0.0,pointroA_stargazer2=0.0;
	
	for (int loop_mark = 0; loop_mark < 100; loop_mark++)
	{
		if (MARK[loop_mark].markID == StarGazer.starID)
		{
			pointrox_stargazer1 = MARK[loop_mark].mark_x+ StarGazer.starX;
			pointroy_stargazer1 = MARK[loop_mark].mark_y + StarGazer.starY;
			pointroA_stargazer1=StarGazer.starAngel-MARK[loop_mark].mark_angle;
		}
		if (MARK[loop_mark].markID == StarGazer.starID2)
		{
			pointrox_stargazer2 = MARK[loop_mark].mark_x+ StarGazer.starX2;
			pointroy_stargazer2 = MARK[loop_mark].mark_y + StarGazer.starY2;
			pointroA_stargazer2=StarGazer.starAngel2-MARK[loop_mark].mark_angle;
		}
	}
	int isone=1;
	if((pointrox_stargazer1<0.001&&pointrox_stargazer1>-0.001&&pointroy_stargazer1>-0.001&&pointroy_stargazer1<0.001)||(pointrox_stargazer2<0.001&&pointrox_stargazer2>-0.001&&pointroy_stargazer2>-0.001&&pointroy_stargazer2<0.001))
	{
		if((pointrox_stargazer1<0.001&&pointrox_stargazer1>-0.001&&pointroy_stargazer1>-0.001&&pointroy_stargazer1<0.001)&&(pointrox_stargazer2<0.001&&pointrox_stargazer2>-0.001&&pointroy_stargazer2>-0.001&&pointroy_stargazer2<0.001))
		{
			isone=0;//xhy:没有为0
		}
		else
		{
			isone=2;//xhy:双星标为1，单星标为2
		}

	}
	///////出界检测
	overx3=overx2;
	overy3=overy2;
	overa3=overa2;

	overx2=overx1;
	overy2=overy1;
	overa2=overa1;

	overx1=StarGazer.starX;
	overy1=StarGazer.starY;
	overa1=pointroA_stargazer1;

	overx32=overx22;
	overy32=overy22;
	overa32=overa22;

	overx22=overx12;
	overy22=overy12;
	overa22=overa12;

	overx12=StarGazer.starX2;
	overy12=StarGazer.starY2;
	overa12=pointroA_stargazer2;


	if(overx32==overx22&&overx22==overx12&&overy32==overy22&&overy22==overy12&&overa32==overa22&&overa22==overa12&&isone!=0)
	{
		if(overx3==overx2&&overx2==overx1&&overy3==overy2&&overy2==overy1&&overa3==overa2&&overa2==overa1)
		{
			isone=0;
		}
		else
		{
			isone=2;
		}
	}
	///////////////
	fprintf(alloutsar," 111===%f        %f    \n",StarGazer.starX,StarGazer.starY);
	fprintf(alloutsar," 222===%f     %f   \n",StarGazer.starX2,StarGazer.starY2);


	fprintf(alloutsar," 3333===%f    %f   %f     %f  %f   \n",StarGazer.starAngel,StarGazer.starID,pointrox_stargazer1,pointroy_stargazer1,pointroA_stargazer1);
	fprintf(alloutsar," 4444===%f    %f   %f     %f  %f   \n",StarGazer.starAngel2,StarGazer.starID2,pointrox_stargazer2,pointroy_stargazer2,pointroA_stargazer2);
	fprintf(alloutsar," 0000===%d    \n",isone);



	float starAngel360;
	float starAngel3601=StarGazer.starAngel,starAngel3602=StarGazer.starAngel2;
	if(starAngel3601>360)
	{
		starAngel3601-=360;
	}
	else if(starAngel3601<0)
	{
		starAngel3601+=360;
	}

	if(starAngel3602>360)
	{
		starAngel3602-=360;
	}
	else if(starAngel3602<0)
	{
		starAngel3602+=360;
	}

	if(isone==2)
	{
			Info_robot.pointrox_stargazer=pointrox_stargazer1;
			Info_robot.pointroy_stargazer=pointroy_stargazer1;
			starAngel360=starAngel3601;

	}
	else
	{
		Info_robot.pointrox_stargazer=(pointrox_stargazer1+pointrox_stargazer2)/2*isone;
		Info_robot.pointroy_stargazer=(pointroy_stargazer1+pointroy_stargazer2)/2*isone;
		starAngel360=(starAngel3602+starAngel3601)/2;
		if(abs(starAngel3602-starAngel3601)>330)
		{
			starAngel360-=180;
		}
		

	}
	if(starAngel360>360)
	{
		starAngel360-=360;
	}
	else if(starAngel360<0)
	{
		starAngel360+=360;
	}
	Info_robot.pianzhuan_stargazer=starAngel360;

	float pointrox_stargazerchange=(10000+2084)-Info_robot.pointrox_stargazer*10;
	float pointroy_stargazerchange=(10000-152)-Info_robot.pointroy_stargazer*10;
	float pointroy_jiaoduchange=360-Info_robot.pianzhuan_stargazer;
	int isok=0;
	if(isone!=0)
	{
		dic3=dic2;
		dic2=dic1;
		dic1=pointrox_stargazerchange;

		if(dic1-dic2<100.0&&dic1-dic2>-100.0&&dic2-dic3<100.0&&dic2-dic3>-100.0)
		{
			if(abs(Info_robot.pointrox-pointrox_stargazerchange)>300||abs(Info_robot.pointroy-pointroy_stargazerchange)>300||(abs(Info_robot.pianzhuan*180/PIf-pointroy_jiaoduchange)>20&&abs(Info_robot.pianzhuan*180/PIf-pointroy_jiaoduchange)<350))
			{
				isok=0;
			}
			else 
			{
				isok=1;
				if(starnew>4)
				{
					starnew=0;
					Info_robot.pointrox = pointrox_stargazerchange;
					Info_robot.pointroy = pointroy_stargazerchange;
					Info_robot.pianzhuan = pointroy_jiaoduchange*PIf/180;
				}
			}
			
		}
	}
	if(isok==0)
	{
		++starmust;
	}
	else
	{
		starmust=0;
		starnew=0;
	}
	if(starmust>6)
	{
		starmust=0;
		if(StarGazer.starID>0.5&&isone!=0)
		{
			Info_robot.pointrox = pointrox_stargazerchange;
			Info_robot.pointroy = pointroy_stargazerchange;
			Info_robot.pianzhuan = pointroy_jiaoduchange*PIf/180;
			if(isone==0)
			{
				plan.SERVEMODE=4;//机器人停止,已处于未知地址
			}
		}

	}
	if(isok==1)
	{
		++starnew;
	}
	fprintf(alloutstar," 5555=  %d =  %d =%f    %f   %f   %f    %f   %f   \n",isone,isok,Info_robot.pointrox_stargazer,Info_robot.pointroy_stargazer,Info_robot.pianzhuan_stargazer,pointrox_stargazerchange,pointroy_stargazerchange,pointroy_jiaoduchange);
	

//	fprintf(allout,"Info_robot= %f     %f       %f    %f   \n",pointrox,pointroy,pianzhuan,dtheta);
	fclose(allout);
	fclose(alloutstar);
	fclose(alloutsar);




}



