#ifndef DriveBase_h
#define DriveBase_h

#include <Arduino.h>
#include <Config2022.h>
#include <Meth.h>
#include <Math.h>
#include <GY2022.h>
#include <Motor2022.h>
#include <Lidar2022.h>

class DriveBase {
	public:
		DriveBase(Motor2022 *FL, Motor2022 *FR, Motor2022 *BL, Motor2022 *BR);//
		//void move(int ang, float speed);
		void correction(double speed, double angle, double rotation);


	private:
		Motor2022 *_FL;// = Motor2022(14, 2);
		Motor2022 *_FR;//= Motor2022(23, 36);
		Motor2022 *_BL;// = Motor2022(6, 5);
		Motor2022 *_BR;// = Motor2022(29, 30);


};


#endif
