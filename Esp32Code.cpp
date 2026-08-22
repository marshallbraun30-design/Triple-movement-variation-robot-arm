
#include <Servo.h>
#include <LiquidCrystal.h>






// Coordinate structure
struct Coordinates
{
	int x;
	int y;
	int z;
};



// Lcd screen 
const int rs = 2;
const int en =4;
const int d4 = 7;
const int d5 = 8;
const int d6 = 12;
const int d7 = 13;

LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

// Robot joints 
Servo shoulder;
Servo wrist;
Servo elbow;
Servo base;


void setup()
{

	// Initial lcd setup
	lcd.begin(16,2);


  // Initial joint setup
  shoulder.attach(5);
	elbow.attach(6);
	wrist.attach(9);
	base.attach(10);

  shoulder.write(90);
  wrist.write(90);
  elbow.write(90);
  base.write(90);


  Serial.begin(115200);

}





void loop()
{
	/* 1-get coordinates 2-test if there valid and in range 3-if not in range call coordinates till valid. 
	4-send values to movement function which will move machine
	*/
  // Initial default angles (setup movedthem to this angle initially)
	int ang1 = 90;
	int ang2 = 90;
	int ang3 = 90;
	int baseAngle = 90;

	while(true)
	{

		Coordinates target = Inputvalues();
    lcd.clear();
    lcd.print("Coordinates finished");
    delay(2000);
    Serial.print("X =");
    Serial.println(target.x);

    Serial.print("Y =");
    Serial.println(target.y);

    Serial.print("Z =");
    Serial.println(target.z);
    
		
		AngCalculation(target.x,target.y,target.z,ang1,ang2,ang3,baseAngle);

    Serial.print("Calculation complete");

		if (fwdtest(ang1,ang2,ang3,target.x,target.y))
		{
			Serial.print("Test passed");
      ArmMovement(ang1,ang2,ang3,baseAngle);
			break;
		}

	}

	
}



Coordinates Inputvalues()
{
	
	Coordinates target;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Input your X value");
  lcd.setCursor(1,0);
  
  target.x = GetCoordinate('X');
  lcd.println(target.x);

  delay(500);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Input your y value");
  lcd.setCursor(1,0);


  target.y = GetCoordinate('Y');
  lcd.println(target.y);

  delay(500);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Input your z value");
  lcd.setCursor(1,0);

  target.z = GetCoordinate('Z');
  lcd.println(target.z);

  delay(500);
  lcd.clear();
  lcd.print("Leaving the input function");

return target;
}

int GetCoordinate(char domain)
{
  
  int ReturnedVal = 0;

  bool confirm = true;

// taking in the info from serial
  while(confirm)
  {
    if (Serial.available() > 0)
      {
        int Value = Serial.parseInt();
				while(Serial.available())
					{
						Serial.read();
					}
					Serial.print("Received: ");
    			Serial.println(Value);

// Clearing the value section
          if (Value == 50)
          {
            ReturnedVal = 0;

          }


// confirming the value section
          else if (Value == 60)
          {

            delay(1000);
            confirm = false;
          }
  
// Inputing a value
          else
          {
            ReturnedVal = Value;

          }
      }
  }
  Serial.println("loop finished");
  return ReturnedVal;



}


bool fwdtest(int ang1, int ang2, int ang3, int x, int y) //This function is done
{
	/* This function will take in the inverse kinematics values and check forward kinematics to make sure its all proper.
	it will also make sure that the position is within the machines limits. it will return a boolean value.*/
  

  // Link lengths on the robot
  int l1 = 30;
  int l2 = 20;
  int l3 = 20;


  float tolerance = 1.0f;
	
	float rad1 = ang1*PI/180;
	float rad2 = ang2*PI/180;
	float rad3 = ang3*PI/180;
 
  float OutcomeX = l1*cos(rad1) + l2 * cos(rad1 + rad2) + l3 * cos(rad1 + rad2 + rad3);
  float OutcomeY = l1*sin(rad1) + l2 * sin(rad1 + rad2) + l3 * sin(rad1 + rad2 + rad3);

  float DifferenceX = fabs(OutcomeX-x);
  float DifferenceY = fabs(OutcomeY-y);

	if (DifferenceX <= tolerance
      && DifferenceY <= tolerance)
		{
			return true;
		}
	else
	  {
		return false;
	  }	
}

void AngCalculation(int targX, int targY, int targz, int &ang1, int &ang2,int &ang3, int &baseAngle) //This function is done
{
	/*
	
	This function serves the purpose of doing the inverse 
	kinematic proces to find the needed angles 
	to reach the desired point. it takes in the desired 
	position then gives the needed angles. It doesnt adjust 
	using current angle. that will be done elsewhere. 

	
	*/

    // Link lengths on the robot
  int l1 = 30;
  int l2 = 20;
  int l3 = 20;

  // Wrist position
  float wristX = 1;
  float wristY = 1;

  // Different parts of the equation will be stored here
  float first = 1;
  float second = 1;
  float third = 1;
  float fourth = 1;
	

	wristX = targX - l3 * cos(1.57);
	wristY = targY - l3 * sin(1.57);


  // Angle one equation 
	first = wristX * wristX + wristY * wristY + l1*l1 - l2*l2;
	second = 2 * l1 * sqrt(wristX*wristX + wristY*wristY);
	ang1 = (atan(wristY / wristX) - acos(first / second))*180/PI;

  // Angle two equation
  third = l1*l1 + l2*l2 - wristX*wristX - wristY*wristY;
	fourth = 2 * l1 * l2;
	ang2 = (1.571 - acos(third / fourth))*180 / PI;

  //Angle 3 equation
	ang3 = 90 - ang1 - ang2;


	baseAngle = atan2(targz,targX)* 180 / PI;
}

void ArmMovement(int ang1, int ang2, int ang3, int baseAngle) // this function is done for now. 
{
	/* 
	This function moves the robot arm
	*/
	base.write(baseAngle);
	if (base.read() == baseAngle)
	{
		shoulder.write(ang1);
		if (shoulder.read() == ang1)
		{
			elbow.write(ang2);
			if (elbow.read() == ang2)
			{
				wrist.write(ang3);
			}
		}
	}
}

