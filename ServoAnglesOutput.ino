#include <math.h>
#include <Servo.h>

Servo myservo[6];

#define pi 3.14159
#define rad_to_deg 180/pi
#define deg_to_rad pi/180
#define z_home  0.238

 float Trans_matrix[3];    //3*1 matrix
 float Rot_matrix[3][3];   //3*3 matrix
 float q[6][3];
 float l[6][3];
 float leg_length[6];
 float servo_angle[6];

// ----------------------declaration of base points------------------------------------------//
 float  r_base = 0.0740;

 float base_point[6][3] = {
    {   -r_base * cos(28*deg_to_rad) ,   -r_base * sin(28*deg_to_rad) , 0.03},
    {   -r_base * cos(28*deg_to_rad) ,    r_base * sin(28*deg_to_rad) , 0.03},
    {    r_base * cos(88*deg_to_rad) ,    r_base * sin(88*deg_to_rad) , 0.03},
    {    r_base * cos(32*deg_to_rad) ,    r_base * sin(32*deg_to_rad) , 0.03},
    {    r_base * cos(32*deg_to_rad) ,   -r_base * sin(32*deg_to_rad) , 0.03},
    {    r_base * cos(88*deg_to_rad) ,   -r_base * sin(88*deg_to_rad) , 0.03}
};
//--------------------end of declaration of base points---------------------------------------//

//-----------------------declaration of platform points----------------------------------------//
 float r_top = 0.0790;

float top_point[6][3] = {
    {   -r_top * cos(pi/4)          ,   -r_top * sin(pi/4)          ,   -0.005},
    {   -r_top * cos(pi/4)          ,    r_top * sin(pi/4)          ,   -0.005},
    {   -r_top * sin(15*deg_to_rad) ,    r_top * cos(15*deg_to_rad) ,   -0.005},
    {    r_top * cos(15*deg_to_rad) ,    r_top * sin(15*deg_to_rad) ,   -0.005},
    {    r_top * cos(15*deg_to_rad) ,   -r_top * sin(15*deg_to_rad) ,   -0.005},
    {   -r_top * sin(15*deg_to_rad) ,   -r_top * cos(15*deg_to_rad) ,   -0.005}
};
//-----------------------end of declaration of platform points ------------------------------//

int zero_pos[6]={90,90,90,90,90,90};    // try to put this in degrees

//initial position of platform centre from base centre

    float trans_init[3] = {0   ,   0   ,  z_home  };       //translation info x_home ,y_home ,z_home  
          
    float rot_init[3]   = {0   ,   0   ,   0      };         //rotation info roll, pitch, yaw


 float beta[6]=  { pi/2 , -pi/2 , -pi/6 , 5*pi/6 , -5*pi/6, pi/6 };               // orientation of servo wrt x axis 
 float Rm = 0.024;               // length of servo arm
 float  D = 0.205;                // length of connecting rod
 
  float req_pos[6] ={0  ,  0  ,  0  ,  0 ,  0,  0 };                       // x y z roll pitch yaw

void getTrans_matrix() {
  
   Trans_matrix[0] = req_pos[0]+trans_init[0];
   Trans_matrix[1] = req_pos[1]+trans_init[1];
   Trans_matrix[2] = req_pos[2]+trans_init[2];
}


void getRot_matrix() {
   
  float phi   = deg_to_rad * req_pos[3];    // roll wrt x axis
  float theta = deg_to_rad * req_pos[4];    // pitch wrt y axis
  float psi   = deg_to_rad * req_pos[5];    // yaw wrt z axis
  
   Rot_matrix[0][0] = cos(psi)*cos(theta);
   Rot_matrix[0][1] = -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
   Rot_matrix[0][2] = sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);

   Rot_matrix[1][0] = sin(psi)*cos(theta);
   Rot_matrix[1][1] = cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
   Rot_matrix[1][2] = -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);

   Rot_matrix[2][0] = -sin(theta);
   Rot_matrix[2][1] = cos(theta)*sin(phi);
   Rot_matrix[2][2] = cos(theta)*cos(phi);
}

// qi is a vector representing the top frame points in the base frame coordinate system
void getQ() {

  for(int i=0;i<6;i++){
  q[i][0]=Trans_matrix[0] + Rot_matrix[0][0]*top_point[i][0] + Rot_matrix[0][1]*top_point[i][1] + Rot_matrix[0][2]*top_point[i][2];
  q[i][1]=Trans_matrix[1] + Rot_matrix[1][0]*top_point[i][0] + Rot_matrix[1][1]*top_point[i][1] + Rot_matrix[1][2]*top_point[i][2];
  q[i][2]=Trans_matrix[2] + Rot_matrix[2][0]*top_point[i][0] + Rot_matrix[2][1]*top_point[i][1] + Rot_matrix[2][2]*top_point[i][2];
 }

}

//************  leg length calculation ****************//
// the leg length is the length between base point and the corresponding top points
void getLeg_length(){

    for(int i=0;i<6;i++){
      l[i][0]=q[i][0]-base_point[i][0];
      l[i][1]=q[i][1]-base_point[i][1];
      l[i][2]=q[i][2]-base_point[i][2];
    }

    for(int i=0;i<6;i++)
    leg_length[i] = sqrt (l[i][0]*l[i][0] + l[i][1]*l[i][1] + l[i][2]*l[i][2]);

}
//**************************************//




//************* servo angle calculation *****************//
void getservo_angle(){
 
 double a[6];
 double b[6];
 double c[6];
 double deno[6];

 for(int i= 0; i<6; i++)
 a[i] = 2*Rm*(q[i][2] - base_point[i][2]);

for(int i= 0; i<6; i++)
 b[i] =2*Rm*((q[i][0] - base_point[i][0])*cos(beta[i]) + (q[i][1] - base_point[i][1])*sin(beta[i]));

for(int i= 0; i<6; i++)
 c[i] = leg_length[i]*leg_length[i] - (D*D) + Rm*Rm;

for(int i=0;i<6;i++)
  deno[i]=sqrt( a[i]*a[i] + b[i]*b[i]);

for(int i=0;i<6;i++)
  servo_angle[i] = rad_to_deg * (asin(c[i]/deno[i]) - atan(b[i]/a[i]));


for(int i=0;i<6; i++){
  if(i==0 || i==2 || i==4)
   myservo[i].write((zero_pos[i]-servo_angle[i]));
  else
   myservo[i].write((zero_pos[i]+servo_angle[i]));
}
}

//*************************************//


void setup() {
myservo[0].attach(3);
myservo[1].attach(5);
myservo[2].attach(6);
myservo[3].attach(9);
myservo[4].attach(10);
myservo[5].attach(11);

for(int i=0 ; i<6 ;i++)
  myservo[i].write(90);

delay(1000);

Serial.begin(9600);
 
}

void loop() {

getTrans_matrix();
getRot_matrix();
getQ();
getLeg_length();

for(int i=0;i<6;i++){
   Serial.print(leg_length[i]);
   Serial.print('\t'); 
} 
Serial.print('\n');

getservo_angle();

 for(int i=0;i<6;i++){
   if(i==0 || i==2  || i==4){
   Serial.print(90-servo_angle[i]);
   Serial.print('\n');
   }
   else{
   Serial.print(90+servo_angle[i]);
   Serial.print('\n');
   }
}

Serial.print('\n');

for(int i=0;i<6;i++){
   if(i==0 || i==2  || i==4)
   myservo[i].write(90-servo_angle[i]);
   else
  myservo[i].write(90+servo_angle[i]);
} 

}