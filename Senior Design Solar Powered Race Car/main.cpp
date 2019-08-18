#include <SoftwareSerial.h>
#include <Arduino.h>
#include <math.h>
#define sind(x) (sin(fmod((x),360) * pi / 180))
#define cosd(x) (cos(fmod((x),360) * pi / 180))
#define L 8
#define pi 3.14159265358979323846
#define NUM 12
#define W 1.2573  //wheel base in m

char inChar=-1;
int l= -1;
int check = 0;
char inData[7]; // Looking for $GNGGA
char latData[3];
char latMinData[10];
char longData[4];
char longMinData[10];

double tempXY[2];
double currXY[2];
double prevXY[2];
double xy[NUM][2];
double slope[NUM];
double yint[NUM];
bool eastwest; // true if east, false if west
double orientation; // slope of the line that represents the kart's orientation
double slopeO, slopeP;
double yintO, yintP;
double angleP, angleO, angleN;
double actualR;
bool abovebelow;
double center[2];		// coordinates of the center of the circle of radius "actualR"
double longitude = 0.0;
double latitude = 0.0;
double tempLat = 0;
double tempLon = 0;

int data=0;
int count= 460;
int flag = 0;
int save = 0;
int i = 0;
int threePings = 0;
int tpCount = 0;
int actuator_move = 0;
SoftwareSerial mySerial(15, 14); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3
SoftwareSerial servoSerial(17, 16); //





double getLatLonDist(double lat1, double long1, double lat2, double long2) {
	double R = 6373042; // in  meters, assuming altitude is 890 m
	double a = pow(sind((lat2 - lat1) / 2), 2);
	// 2*R*asin(sqrt(sind((lat2 - lat1)/2)^2+cosd(lat1)*cosd(lat2)*sind((lon2-lon1)/2)^2))
	double b = cosd(lat1)*cosd(lat2);
	double c = pow(sind((long2 - long1) / 2), 2);
	double d = 2*R*asin(sqrt(a + b*c));   // found this formula to be accurate within a cm
	return d;
}
double getDist(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
double QuadFormPos(double a, double b, double c) {
	return (-b + sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
}
double QuadFormMin(double a, double b, double c) {
	return (-b - sqrt(pow(b, 2) - 4 * a*c)) / (2 * a);
}
void getXY(double xy[], double latitude, double longitude) {
	double latlongO[2]; // origin coordinates
	latlongO[0] = 32.03580519; //origin latitude
	latlongO[1] = -110.79447621; //origin longitude
								 // Set the (0, 0) point to be latitude = 32.03580519 degrees, longitude = -110.79447621 degrees
								 // Calculate distance between the two coordinates
	xy[0] = getLatLonDist(latlongO[0], latlongO[1], latlongO[0], longitude); // x value
	xy[1] = getLatLonDist(latlongO[0], latlongO[1], latitude, latlongO[1]); // y value
																	// if longitude < lon0, x is negative, if latitude < lat0, y is negative
	if (longitude < latlongO[1]) {
		xy[0] = -xy[0];
	}
	if (latitude < latlongO[0]) {
	xy[1] = -xy[1];
  }
  return;
}
double getSlope(double xy1[], double xy2[]) {
	return (xy2[1] - xy1[1]) / (xy2[0] - xy1[0]);
}
double getAngle(double xy1[], double xy2[], double slope, bool eastwest) {
	if (eastwest) {	// if going east, orientation is in right half plane with respect to the current position xy1
		if (slope > 0) {	// orientation is in first quad
			if (xy2[0] > xy1[0]) {	// xy2 is in right half plane
				// xy2 is in first quadrant or 4th quad
				if (xy2[1] > xy1[1]) {	// xy2 is in 1st
					return abs(atan(getSlope(xy1, xy2)) - atan(slope));
				}
				else {					// xy2 is in 4th quad
					return -atan(getSlope(xy1, xy2)) + atan(slope);
				}

			}
			else {	// xy2 is in left half plane
				if (xy2[1] > xy1[1]) {	// xy2 is in 2nd quad
					return (pi + atan(getSlope(xy1, xy2))) - atan(slope);
				}
				else {					// xy2 is in 3rd quad
					return (pi + atan(getSlope(xy1, xy2))) - atan(slope);
				}
			}
		}
		else {	// orientation is in 4th quad
			if (xy2[0] > xy1[0]) {	// xy2 is in right half plane
				if (xy2[1] > xy1[1]) {	// xy2 is in 1st
					return  atan(getSlope(xy1, xy2)) - atan(slope);
				}
				else {					// xy2 is in 4th quad
					return abs(atan(getSlope(xy1, xy2)) - atan(slope));
				}
			}
			else {	// xy2 is in left half plane
				if (xy2[1] > xy1[1]) {	// xy2 is in 2nd quad
					return (pi + atan(getSlope(xy1, xy2))) - atan(slope);
				}
				else {					// xy2 is in 3rd quad
					return (2 * pi - abs(atan(slope))) - (pi + atan(getSlope(xy1, xy2)));
				}
			}
		}
	}
	else {	// if going west, orientation is in left half plane with respect to the current position xy1
		if (slope > 0) {	// orientation is in 3rd quad
			if (xy2[0] > xy1[0]) {	// xy2 is in right half plane
									// xy2 is in first quadrant or 4th quad
				if (xy2[1] > xy1[1]) {	// xy2 is in 1st
					return (pi + atan(slope)) - atan(getSlope(xy1, xy2));
				}
				else {					// xy2 is in 4th quad
					return (2*pi+atan(getSlope(xy1, xy2))) - (pi + atan(slope));
				}

			}
			else {	// xy2 is in left half plane
				if (xy2[1] > xy1[1]) {	// xy2 is in 2nd quad
					return (pi + atan(slope)) - (pi + atan(getSlope(xy1, xy2)));
				}
				else {					// xy2 is in 3rd quad
					return abs(atan(getSlope(xy1, xy2)) - atan(slope));
				}
			}
		}
		else {	// orientation is in 2nd quad
			if (xy2[0] > xy1[0]) {	// xy2 is in right half plane
				if (xy2[1] > xy1[1]) {	// xy2 is in 1st
					return  (pi+atan(slope)) - atan(getSlope(xy1, xy2));
				}
				else {					// xy2 is in 4th quad
					return (2*pi + atan(getSlope(xy1, xy2))) - (pi + atan(slope));
				}
			}
			else {	// xy2 is in left half plane
				if (xy2[1] > xy1[1]) {	// xy2 is in 2nd quad
					return abs(atan(getSlope(xy1, xy2)) - atan(slope));
				}
				else {					// xy2 is in 3rd quad
					return pi + atan(getSlope(xy1, xy2)) - (pi + atan(slope));
				}
			}
		}
	}
}
void getSlopes(double xy[][2], double slope[]) {
	int i = 0;
	for (i = 0; i < NUM; i = i + 1) {
		if (i < NUM-1) {
			slope[i] = getSlope(xy[i], xy[i + 1]);
		}
		else {
			slope[i] = getSlope(xy[i], xy[0]);
		}
	}
}
double getYInt(double xy1[], double slope) {
	return xy1[1] - slope*xy1[0];
}
bool AboveOrBelow(double la[], double orientation, double currXY[]) {

	double yint = getYInt(currXY, orientation);

	if (la[1] >= (la[0] * orientation + yint)) {
		return true; // Above
	}
	else {
		return false; // Below
	}
}
void getYInts(double xy[][2], double slope[], double yint[]) {
	int i = 0;
	for (i = 0; i < NUM; i = i + 1) {
		yint[i] = getYInt(xy[i], slope[i]);
		yint[i] = getYInt(xy[i], slope[i]);
	}
}
double findSlope(double angle) {// angle from 0 - 2pi
	double newAng = angle;
	double slope;
	if (angle > pi && angle < 2*pi) {
		newAng = angle - pi;
	}
	slope = tan(newAng);
	return slope;
}
double findCircleCenter(double currXY[], double r, double m, double yintP, bool above) {
	double a = 1 + pow(m, 2);
	double b = -2 * currXY[0] + 2 * m*yintP - 2 * m*currXY[1];
	double c = pow(currXY[0], 2) + pow(yintP, 2) - 2 * yintP * currXY[1] - pow(r, 2) + pow(currXY[1], 2);

	double x1 = QuadFormMin(a, b, c);
	double x2 = QuadFormPos(a, b, c);

	if (above) {
		if (m*x1 + yintP > currXY[1]) {
			return x1;
		}
		else {
			return x2;
		}
	}
	if (!above) {
		if (m*x1 + yintP < currXY[1]) {
			return x1;
		}
		else {
			return x2;
		}
	}
}
double getNearestDist(double slope[], double yint[], int num, double currXY[]) {
	int g = 0;
	double nearestDist;
	double nearestDistX;
	double d = 100;
	for (g = 0; g < num; g++) {
		nearestDistX = (currXY[0] - yint[g] * slope[g] + currXY[1] * slope[g]) / (1 + pow(slope[g], 2));
		nearestDist = sqrt(pow((currXY[0] - nearestDistX), 2) + pow((currXY[1] - (slope[g] * nearestDistX + yint[g])), 2));
		if (nearestDist < d) {
			d = nearestDist;
		}
	}
	return d;
}
void delayMill(int milli) {
	long unsigned startTime = millis();
	long unsigned endTime = startTime + milli;
	while (endTime > millis()) {

	}
	return;
}



  void RF_VT() // interrupt service function
{
  data=(digitalRead(4)<<3)+(digitalRead(5)<<2)+(digitalRead(6)<<1)
                                            +(digitalRead(7)<<0);
  flag++;
  if(flag == 200)
  {
    flag = 0;
  }
  // Serial.print("Data: ");
  // Serial.print(data);
  // Serial.print("\tThreePings: ");
  // Serial.print(threePings);
  // Serial.print("\tCount: ");
  // Serial.println(count);
}

void GPS() {
	inChar=-1; // Where to store the character read
	if (Serial1.available()) {
		inChar = Serial1.read();
		if (inChar == '$'){
			l= 0;
			inData[l] = inChar;
		}
		else if (l>= 0 && l< 6) {
			l++;
			inData[l] = inChar;
		}
		else if (l== 6) {
			inData[l] = '\0';
			l++;
		}
		if (strcmp(inData, "$GNGLL") == 0) {
			if (l>= 7 && l< 17) {
				if (l== 7) {
					latData[0] = inChar;
				}
				if (l== 8) {
					latData[1] = inChar;
					latData[2] = '\0';
				}
				else {
					latMinData[l-9] = inChar;
				}
				l++;
			}
			else if (l== 17) {
				latMinData[l-8] = '\0';
				l++;
				// Serial.println(latData);
				latitude = atof(latData) + atof(latMinData)/60;
			}
			else if (l> 17 && l< 20) {
				l++;
			}
			else if (l>= 20 && l< 23) {
				longData[l-20] = inChar;
				l++;
				if (l== 23) {
					longData[3] = '\0';
				}
			}
			else if (l>= 23 && l< 31) {
				longMinData[l-23] = inChar;
				l++;
			}
			else if (l== 31) {
				longMinData[l-23] = '\0';
				l= -1;
				longitude = atof(longData) + atof(longMinData)/60;
			}
		}
		// Serial.println("Latitude: ");
		// Serial.println(latitude);
	}
	check = 1;
}

//sets the new target for the JRK21V3 controller, this uses pololu high resulution protocal
void Move(int x) {
  word target = x;  //only pass this ints, i tried doing math in this and the remainder error screwed something up
  mySerial.write(0xAA); //tells the controller we're starting to send it commands
  mySerial.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  mySerial.write(0x40 + (target & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  mySerial.write((target >> 5) & 0x7F);   //second half of the target, " " "
}

void setup()
{

  Serial.begin(9600);
  Serial1.begin(19200);   // GPS Initialization

  servoSerial.begin(9600);
  while (!mySerial) {
  }
  //delay(3000);
	delayMill(3000);

	Serial.println(1);
  servoSerial.println("Y");
  // delay(3000);
	delayMill(3000);

	Serial.println(2);
  servoSerial.println("X");
  // delay(3000);
	delayMill(3000);

	Serial.println(3);
  servoSerial.println("P0");
  // delay(3000);
	delayMill(3000);

	Serial.println(4);
  servoSerial.println("M255");
  // delay(500);
	delayMill(500);

	Serial.println(5);
  servoSerial.println("D100");
  // delay(500);
	delayMill(500);

	Serial.println(6);
  servoSerial.println("S0");
//killswitch setup
pinMode(3,INPUT);
pinMode(4,INPUT);
pinMode(5,INPUT);
pinMode(6,INPUT);
pinMode(7,INPUT);
pinMode(8,OUTPUT);
attachInterrupt(1 ,RF_VT,RISING);
attachInterrupt(digitalPinToInterrupt(19), GPS, RISING);


//actuator setup
mySerial.begin(9600);
//SoftwareSerial mySerial(15, 14); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3

getXY(xy[0], 32.03585129, -110.79445988);	  // (320, 320)
getXY(xy[1], 32.03585114, -110.79566249);   // (96, 320) in u-center view 2, midline
getXY(xy[2], 32.03578201, -110.79604368);	  // (25, 335)
getXY(xy[3], 32.03561217, -110.79642482);	  // (327, 259)
getXY(xy[4], 32.03545233, -110.79655802);   // (302, 294)
getXY(xy[5], 32.03533050, -110.79643968);   // (324, 321)
getXY(xy[6], 32.03536565, -110.79626352);   // (357, 313)
getXY(xy[7], 32.03553600, -110.79557886);   // (485, 276)
getXY(xy[8], 32.03546681, -110.79487398);   // (327, 322)
getXY(xy[9], 32.03543066, -110.79458865);   // (381, 330)
getXY(xy[10], 32.03560483, -110.79430197);   // (434, 291)
getXY(xy[11], 32.03574585, -110.79430117);   // (434, 260)

getXY(currXY, 32.03585096, -110.79445061);
Serial.println(currXY[0]);
Serial.println(currXY[1]);
orientation = 0.0001;
// Serial.println(orientation);
eastwest = false;
abovebelow = true;
actualR = 1000;
slopeO = orientation;	// orientation slope
slopeP = -1 / slopeO;				// perpendicular slope
yintO = getYInt(currXY, slopeO);
yintP = getYInt(currXY, slopeP);
angleO = abs(atan(slopeO));
angleP = abs(atan(slopeP));
center[0] = findCircleCenter(currXY, actualR, slopeP, yintP, 1);
center[1] = slopeP*center[0] + yintP;
getSlopes(xy, slope);
getYInts(xy, slope, yint);
prevXY[0] = currXY[0];
prevXY[1] = currXY[1];

}


void loop()
{
// Serial.print(count);
// Serial.print(" ");
// Serial.print(i);
// Serial.print(" ");
// Serial.println(latitude);
// Serial.println(longitude);
// Serial.println();
// Serial.print("Orientation: ");
// Serial.print(orientation);
// Serial.println();
  count++;                // number of cycles the on board arduino
	// Serial.println(latitude);
	// Serial.println(longitude);                       // has done since the last transmition
  save = flag;            // saves value of data (value the kill switch transmitter sent)
  delayMill(10);

  if(data == 2){          // when data is the "go" value (2)
    threePings++;
    if(threePings >= 3){  // check that 3 transmitions have been
                          // recieved before restarting kart
      threePings = 3;     // reset value of three pings received to 3
      count = 0;          // reset the counter to zero
    }
  }
  // Serial.println(count);                        // it takes 60 cycles to receive one transmition
  if(count < 300){        // as long as the count is below 130 cycles
    i++;                  // accelerator speed value
    if(i >= 650){
      i = 650;
      // change i value to control speed of kart
      // higher i means faster kart, cannot exceed ~4V
    }
    analogWrite(8,i/5);  // slowly speed up the accelerator
   /*****************************
    * this is where the code goes for the kart moving
    *****************************/
    if(actuator_move == 1){
      Move(0);
      Serial.println("setting actuator to zero position");
      actuator_move = 0;
    }
  /// Enter code for kart logic

/****************************************************************************
 *
 * This is where the code will go for when the kart is moving forward
 *
 ****************************************************************************/
//=================================================================================

// if (tempLat != latitude || tempLon != longitude) {
	// tempLat = latitude;
	// tempLon = longitude;
	// getXY(tempXY, tempLat, -tempLon);   // Update the current position

	// prevXY[0] = currXY[0];
	// prevXY[1] = currXY[1];
	getXY(currXY, latitude, -longitude);   // Update the current position
	// Serial.print(latitude);
	// Serial.print(" ");
	// Serial.print(-longitude);
	// Serial.println();
	// Serial.print(currXY[0]);
	// Serial.print(" ");
	// Serial.print(currXY[1]);
	// Serial.println();
	double dist = getDist(currXY[0], currXY[1], prevXY[0], prevXY[1]);
	Serial.println(dist);
	Serial.print("x: ");
	Serial.print(currXY[0]);
	Serial.print(" y: ");
	Serial.print(currXY[1]);
	Serial.println();
	if (getDist(currXY[0], currXY[1], prevXY[0], prevXY[1]) > 1) {
		Serial.print(currXY[0]);
		Serial.print(" ");
		Serial.print(currXY[1]);
		Serial.println();
		prevXY[0] = currXY[0];
		prevXY[1] = currXY[1];
		// getXY(currXY, tempLat, -tempLon);   // Update the current position
		orientation = -1/getSlope(currXY, center);	// Update the orientation
		Serial.println(orientation);
		if (abovebelow) {		// if circle is above orientation line
			if (!eastwest) {	// if going west
				// going clockwise
				if (currXY[1] > center[1]) {	// if we're on the top half of the circle
					eastwest = true;	// Going east if on top of clockwise circle
				}
				else {
					eastwest = false;	// west on bottom of cw circle
				}
			}
			else {						// if going east
				// going counterclockwise
				if (currXY[1] > center[1]) {	// if we're on the top half of the circle
					eastwest = false;	// Going west if on top of ccw circle
				}
				else {
					eastwest = true;	// east on bottom of ccw circle
				}
			}
		}
		else {						// if circle is below orientation line
			if (!eastwest) {
				// counterclockwise
				if (currXY[1] > center[1]) {	// if we're on the top half of the circle
					eastwest = false;	// Going west if on top of ccw circle
				}
				else {
					eastwest = true;	// east on bottom of ccw circle
				}
			}
			else {
				// clockwise
				if (currXY[1] > center[1]) {	// if we're on the top half of the circle
					eastwest = true;	// Going east if on top of clockwise circle
				}
				else {
					eastwest = false;	// west on bottom of cw circle
				}
			}
		}
		// orientation, position, and direction have been determined
		// Serial.println(eastwest);
		double a;
		double b;
		double c;
		double nearestDistX;
		double nearestDist;
		int f;
		double laX1, laX2, laX;
		double la[2];
		la[0] = 100000;
		double bigX, littleX;
		double angleLAOr, angleXY, perpDist, r;
		int servoNum;
		for (f = 0; f < NUM; f = f + 1) {
			a = 1 + pow(slope[f], 2);
			b = -2 * currXY[0] + 2 * slope[f] * yint[f] - 2 * slope[f] * currXY[1];
			c = pow(currXY[0], 2) + pow(yint[f], 2) - 2 * yint[f] * currXY[1] - pow(L, 2) + pow(currXY[1], 2);
			nearestDistX = (currXY[0] - yint[f] * slope[f] + currXY[1] * slope[f]) / (1 + pow(slope[f], 2));
			nearestDist = sqrt(pow((currXY[0] - nearestDistX), 2) + pow((currXY[1] - (slope[f] * nearestDistX + yint[f])), 2));
			if (nearestDist < L) {
				laX1 = QuadFormPos(a, b, c);	// greater x value
				laX2 = QuadFormMin(a, b, c);
				if (f < NUM) {
					bool greaterthannext;
					greaterthannext = (xy[f + 1][0] < xy[f][0]);
					if (greaterthannext) {
						bigX = xy[f][0];
						littleX = xy[f + 1][0];
						laX = laX2;
					}
					else {
						bigX = xy[f + 1][0];
						littleX = xy[f][0];
						laX = laX1;
					}
				}
				else {
					bool greaterthannext = (xy[0][0] < xy[f][0]);
					if (greaterthannext) {
						bigX = xy[f][0];
						littleX = xy[0][0];
						laX = laX2;
					}
					else {
						bigX = xy[0][0];
						littleX = xy[f][0];
						laX = laX1;
					}
			}

				// laX should be in domain of the current line

				if (laX < bigX && laX > littleX) {
					la[0] = laX;
					la[1] = slope[f] * la[0] + yint[f];
					break;
				}
			}
		}

			if (la[0] == 100000 || getNearestDist(slope, yint, NUM, currXY) > 3) {
				//STOPPPPPPP THE KARTTTTTTTTT
				// stop motor, extend actuator
				count = 500;
				Serial.println("Stopping kart, too far away");
				//printf("Stop the kart, it's too far off the track\n");
			}
			else {
				/* Once lookahead distance is found, we must find the perpendicular and parallel distance components.
				*/
				Serial.println("within bounds of track");
				// Serial.print("lookahead: (");
				// Serial.print(la[0]);
				// Serial.print(", ");
				// Serial.print(la[1]);
				// Serial.print(")");
				// Serial.println();
				// lookahead angle with respect to the kart's orientation: (arctan(slope between currXY and prevXY) - arctan(slope between currXY and prevXY))

				angleLAOr = getAngle(currXY, la, orientation, eastwest);
				angleXY = pi / 2 - angleLAOr;
				Serial.print("angle of lookahead and orientation ");
				Serial.print(angleLAOr);
				perpDist = cos(angleXY)*L;

				r = pow(L, 2) / (2 * perpDist);
				if (r < 3.5) {
					r = 3.5;
				}
				slopeO = orientation;	// orientation slope
				slopeP = -1 / slopeO;				// perpendicular slope
				yintO = getYInt(currXY, slopeO);
				yintP = getYInt(currXY, slopeP);

				servoNum = round((atan(W/r)*180/pi)/0.0138);		// .0148
				if (servoNum > 600) {
					servoNum = 600;
				}
				actualR = W/tan((0.0048*servoNum)*pi/180);
				if (servoNum == 0) {
					actualR = 100000;
				}

				abovebelow = AboveOrBelow(la, orientation, currXY);
				if (abovebelow) {

					// circle will be above orientation line
					center[0] = findCircleCenter(currXY, actualR, slopeP, yintP, 1);
					center[1] = slopeP*center[0] + yintP;
					if (!eastwest) {
						// going right/clockwise
						//Servonum is positive
						// Turn Servo right

					}
					else {
						// going left/ccw
						servoNum = -servoNum;
						// turn servo left

					}
				}
				else {
					// circle will be below the orientation line
					center[0] = findCircleCenter(currXY, actualR, slopeP, yintP, 0);
					center[1] = slopeP*center[0] + yintP;
					if (!eastwest) {
						// going left/ccw
						servoNum = -servoNum;
						//Turn servo left


						// angleN = fmod(angleP + theta, 2 * pi);
						// if (angleN < pi && angleN > 0) {
						//   // upper half
						//   eastwest = false;
						// }
						// else {
						//   //bottom half
						//   eastwest = true;
						// }
					}
					else {
						// going right/clockwise
						// servoNum is positive
						// Turn Servo right

					}
				}
				char buffer[15];
				char servoChar[8] = "G";
				sprintf(buffer, "%d", servoNum);
				strcat(servoChar, buffer);
				// Serial.println(eastwest);
				Serial.println(r);
				Serial.println(servoChar);
				servoSerial.println(servoChar);					// Should work
				delayMill(500);
	}

}

}

//================================================================================
    /// end code for kart logic
  //end kart moving forward

else{                   // if two transmitions are missed
    digitalWrite(8,0);    // stop acceleration

 /****************************************************************************
  *
  * This is where the code will go for when the kart is stopped
  *
  ****************************************************************************/
 //===============================================================================



  //servoSerial.println("G0");
  //delay(500);








 //===============================================================================
    if(actuator_move == 0){
      Move(192);
      Serial.println("extending actuator");
      actuator_move = 1;
    }
    i=0;                  // reset acceleration
    if(threePings > 2){
       threePings = 0;       // resets number of pings received
    }
  }//end kart stop

  if(flag == save){       // checks to see if data is the same
                          // number as before
    data = 0;             //resets data to 0
  }

  if(count >= 1000){       // makes sure the counter doesn't rise
                          // to an unreasonable number
    count = 450;
    threePings = 0;       // resets number of pings received

  }
//end main loop
}



//**************************************************************************
