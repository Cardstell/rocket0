#include <Wire.h>
#include <Kalman.h>
#include <SoftwareSerial.h>
//#define CALIB

Kalman kalmanX, kalmanY;
SoftwareSerial softserial(2, 3);
uint8_t IMUAddress = 0x68;
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ, tempRaw;
int16_t offsets[6] =  {-740, -380, -1300, 0, 0, 0};
double cfkalib[6] = {1691.8, 1680.1, 1683.6, 1.0, 1.0, 1.0};
uint32_t mctimer, start;
double angleX, angleY;
boolean gpsStatus[] = {false, false, false, false, false, false, false};
double prevLat = 56.34, prevLon = 60.54, prevAlt = 200, Hspeed = 0, Vspeed = 0;
double speeds[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
uint32_t countSat = 0, cnloop = 0;

const double RAD = 3.14159265 / 180.0;
const double rEarth = 6371000;
const int countUpdates = 50;
const uint32_t mod = 1000003;

void i2cWrite(uint8_t registerAddress, uint8_t data){
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
	uint8_t data[nbytes];
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(IMUAddress, nbytes);
	for(uint8_t i = 0; i < nbytes; i++)
		data [i]= Wire.read();
	return data;
}

void updateMPU() {
	int32_t daccX = 0, daccY = 0, daccZ = 0, dtempRaw = 0, 
		dgyroX = 0, dgyroY = 0, dgyroZ = 0;
	for (int i = 0;i<countUpdates;++i) {
		uint8_t *data = i2cRead(0x3B,14);
		daccX += ((data[0] << 8) | data[1]);
		daccY += ((data[2] << 8) | data[3]);
		daccZ += ((data[4] << 8) | data[5]);
		dtempRaw += ((data[6] << 8) | data[7]);
		dgyroX += ((data[8] << 8) | data[9]);
		dgyroY += ((data[10] << 8) | data[11]);
		dgyroZ += ((data[12] << 8) | data[13]);
		daccX += offsets[0];
		daccY += offsets[1];
		daccZ += offsets[2];
	}
	accX = daccX / countUpdates;
	accY = daccY / countUpdates;
	accZ = daccZ / countUpdates;
	tempRaw = tempRaw / countUpdates;
	gyroX = dgyroX / countUpdates;
	gyroY = dgyroY / countUpdates;
	gyroZ = dgyroZ / countUpdates;
}

/*GPS Settings:
[0]NavMode, [1]DataRate1, [2]DataRate2, [3]PortRateByte1, [4]PortRateByte2, [5]PortRateByte3,
[6]NMEA GLL Sentence, [7]NMEA GSA Sentence, [8]NMEA GSV Sentence, [9]NMEA RMC Sentence, [10]NMEA VTG Sentence
NavMode:
Pedestrian Mode    = 0x03
Automotive Mode    = 0x04
Sea Mode           = 0x05
Airborne < 1G Mode = 0x06

DataRate:
1Hz     = 0xE8 0x03
2Hz     = 0xF4 0x01
3.33Hz  = 0x2C 0x01
4Hz     = 0xFA 0x00

PortRate:
4800   = C0 12 00
9600   = 80 25 00
19200  = 00 4B 00  **SOFTWAREsoftserial LIMIT FOR UNO R3**
38400  = 00 96 00  **SOFTWAREsoftserial LIMIT FOR MEGA 2560!**
57600  = 00 E1 00
115200 = 00 C2 01
230400 = 00 84 03

NMEA Messages:
OFF = 0x00
ON  = 0x01*/

void setup() {
	Wire.begin();
	softserial.begin(9600);
	Serial.begin(9600);
	i2cWrite(0x6B, 0x00);
	for (int i = 0;i<100;++i) i2cRead(0x3B,14);
	byte settingsArray[] = {0x04, 0xFA, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	configureUblox(settingsArray);
	mctimer = micros();
}

void loop() {
	updateMPU();
	double d_aX = accX / cfkalib[0];
	double d_aY = accY / cfkalib[1];
	double d_aZ = accZ / cfkalib[2];
	double acc = sqrt(pow(d_aX, 2) + pow(d_aY, 2) + pow(d_aZ, 2));
	double accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
	double accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;  
	double gyroXrate = (double)gyroX/131.0;
	double gyroYrate = -((double)gyroY/131.0);
	//gyroXangle += kalmanX.getRate()*((double)(micros()-mctimer)/1000000);
	//gyroYangle += kalmanY.getRate()*((double)(micros()-mctimer)/1000000);
	angleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-mctimer)/1000000) - 180.0;
	angleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-mctimer)/1000000) - 180.0;
	mctimer = micros();
	while (Serial.available()) {
		char msg[256];
		int msgsize = 0;
		for (int i = 0;i<256;++i) {
			while (!Serial.available()) {delayMicroseconds(20);}
			msg[i] = Serial.read();
			if (msg[i] =='\n' || msg[i]=='-') {
				msg[i] = 0;
				msgsize = i;
				break;
			}
		}
		if (msgsize<7) continue;
		if (msg[5]=='A' && msg[6]==',') {
			char buff[15][20];
			int prevInd = 0, cur = 0;
			for (int i = 0;i<msgsize;++i) {
				buff[cur][i-prevInd] = msg[i];
				if (msg[i] == ',') {
					buff[cur][i-prevInd] = 0;
					cur++;
					prevInd = i+1;
				}
			}
			if (buff[2][0] != 0 && buff[4][0] != 0 && buff[9][0] != 0) {
				double dlat = atof(buff[2]) / 100.0;
				double dlon = atof(buff[4]) / 100.0;
				double lat = (int)(dlat) + (dlat - (int)(dlat))*5.0/3.0;
				double lon = (int)(dlon) + (dlon - (int)(dlon))*5.0/3.0;
				double alt = atof(buff[9]);
				double x1 = cos(lat*RAD)*rEarth*sin(lon*RAD);
				double y1 = cos(lat*RAD)*rEarth*cos(lon*RAD);
				double z1 = sin(lat*RAD)*rEarth;
				double x2 = cos(prevLat*RAD)*rEarth*sin(prevLon*RAD);
				double y2 = cos(prevLat*RAD)*rEarth*cos(prevLon*RAD);
				double z2 = sin(prevLat*RAD)*rEarth;
				double curHspeed = sqrt(pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2))*4;
				if (curHspeed < 50000) {
					speeds[cnloop%4] = curHspeed;
					speeds[cnloop%4+4] = (alt - prevAlt)*4;
					Hspeed = 0; Vspeed = 0;
					for (int i = 0;i<4;++i) {
						Hspeed += speeds[i];
						Vspeed += speeds[i+4];
					}
					Hspeed /= 4; Vspeed /= 4;
					cnloop++;
				}
				countSat = atoi(buff[7]);
				prevLat = lat;prevLon = lon;prevAlt = alt;
			}
		}
	}
	softserial.print("$> ");
	#ifndef CALIB
	uint32_t checksum = (uint32_t)(acc*1e4) + (uint32_t)(angleX*1e4) + (uint32_t)(angleY*1e4) + countSat;
	checksum %= mod;
	softserial.print(acc, 4);
	softserial.print(" ");
	softserial.print(angleX, 4);
	softserial.print(" ");
	softserial.print(angleY, 4);
	softserial.print(" ");
	softserial.print(countSat, DEC);
	if (countSat != 0) {
		softserial.print(" ");
		softserial.print(prevLat, 7);
		softserial.print(" ");
		softserial.print(prevLon, 7);
		softserial.print(" ");
		softserial.print(prevAlt, 0);
		softserial.print(" ");
		softserial.print(Hspeed, 1);
		softserial.print(" ");
		softserial.print(Vspeed, 1);
		checksum += (uint32_t)(prevLat*1e7)%mod + (uint32_t)(prevLon*1e7)%mod;
		checksum += (uint32_t)(prevAlt) + (uint32_t)(Hspeed*10) + (uint32_t)(Vspeed*10);
		checksum %= mod;
	}
	softserial.print(" ");
	softserial.print(checksum, DEC);
	#else
	softserial.print(accX, DEC);
	softserial.print(" ");
	softserial.print(accY, DEC);
	softserial.print(" ");
	softserial.print(accZ, DEC);
	softserial.print(" ");
	softserial.print(gyroX, DEC);
	softserial.print(" ");
	softserial.print(gyroY, DEC);
	softserial.print(" ");
	softserial.print(gyroZ, DEC); 
	#endif
	softserial.println();
}

void configureUblox(byte *settingsArrayPointer) {
	byte gpsSetSuccess = 0;
	//Configuring GPS
	byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	calcChecksum(&setNav[2], sizeof(setNav) - 4);
	byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
	calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);
	byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);
	byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
	byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
	byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
	byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
	byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
	delay(2500);
	while(gpsSetSuccess < 3)
	{
		//Setting NavMode
		sendUBX(&setNav[0], sizeof(setNav));
		gpsSetSuccess += getUBX_ACK(&setNav[2]);
		if (gpsSetSuccess == 5) {
			gpsSetSuccess -= 4;
			setBaud(settingsArrayPointer[4]);
			delay(1500);
			byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
			sendUBX(lowerPortRate, sizeof(lowerPortRate));
			Serial.begin(9600);
			delay(2000);
		}
		if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
		if (gpsSetSuccess == 10) gpsStatus[0] = true;
	}
	//if (gpsSetSuccess == 3) softserial.println("NavMode configuration failed");
	gpsSetSuccess = 0;
	while(gpsSetSuccess < 3) {
		//Setting data update rate
		sendUBX(&setDataRate[0], sizeof(setDataRate));
		gpsSetSuccess += getUBX_ACK(&setDataRate[2]);
		if (gpsSetSuccess == 10) gpsStatus[1] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	//if (gpsSetSuccess == 3) softserial.println("Data pdate mode configuration failed");
	gpsSetSuccess = 0;
	while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
		//Deactivating NMEA GLL Messages
		sendUBX(setGLL, sizeof(setGLL));
		gpsSetSuccess += getUBX_ACK(&setGLL[2]);
		if (gpsSetSuccess == 10) gpsStatus[2] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	//if (gpsSetSuccess == 3) softserial.println("NMEA GLL Message Deactivation Failed!");
	gpsSetSuccess = 0;
	while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
		//Deactivating NMEA GSA Messages
		sendUBX(setGSA, sizeof(setGSA));
		gpsSetSuccess += getUBX_ACK(&setGSA[2]);
		if (gpsSetSuccess == 10) gpsStatus[3] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	//if (gpsSetSuccess == 3) softserial.println("NMEA GSA Message Deactivation Failed!");
	gpsSetSuccess = 0;
	while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
		//Deactivating NMEA GSV Messages
		sendUBX(setGSV, sizeof(setGSV));
		gpsSetSuccess += getUBX_ACK(&setGSV[2]);
		if (gpsSetSuccess == 10) gpsStatus[4] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	//if (gpsSetSuccess == 3) softserial.println("NMEA GSV Message Deactivation Failed!");
	gpsSetSuccess = 0;
	while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
		//Deactivating NMEA RMC Messages
		sendUBX(setRMC, sizeof(setRMC));
		gpsSetSuccess += getUBX_ACK(&setRMC[2]);
		if (gpsSetSuccess == 10) gpsStatus[5] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	//if (gpsSetSuccess == 3) softserial.println("NMEA RMC Message Deactivation Failed!");
	gpsSetSuccess = 0;
	while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
		//Deactivating NMEA VTG Messages
		sendUBX(setVTG, sizeof(setVTG));
		gpsSetSuccess += getUBX_ACK(&setVTG[2]);
		if (gpsSetSuccess == 10) gpsStatus[6] = true;
		if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
	}
	//if (gpsSetSuccess == 3) softserial.println("NMEA VTG Message Deactivation Failed!");
	gpsSetSuccess = 0;
	if (settingsArrayPointer[4] != 0x25) {
		//Setting port baudrate
		sendUBX(&setPortRate[0], sizeof(setPortRate));
		setBaud(settingsArrayPointer[4]);
		//softserial.println("Success!");
		delay(500);
	}
}


void calcChecksum(byte *checksumPayload, byte payloadSize) {
	byte CK_A = 0, CK_B = 0;
	for (int i = 0; i < payloadSize ;i++) {
		CK_A = CK_A + *checksumPayload;
		CK_B = CK_B + CK_A;
		checksumPayload++;
	}
	*checksumPayload = CK_A;
	checksumPayload++;
	*checksumPayload = CK_B;
}

void sendUBX(byte *UBXmsg, byte msgLength) {
	for(int i = 0; i < msgLength; i++) {
		Serial.write(UBXmsg[i]);
		Serial.flush();
	}
	Serial.println();
	Serial.flush();
}


byte getUBX_ACK(byte *msgID) {
	byte CK_A = 0, CK_B = 0;
	byte incoming_char;
	boolean headerReceived = false;
	unsigned long ackWait = millis();
	byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	int i = 0;
	while (1) {
		if (Serial.available()) {
			incoming_char = Serial.read();
			if (incoming_char == ackPacket[i]) {
				i++;
			}
			else if (i > 2) {
				ackPacket[i] = incoming_char;
				i++;
			}
		}
		if (i > 9) break;
		if ((millis() - ackWait) > 1500) {
			//softserial.println("ACK Timeout");
			return 5;
		}
		if (i == 4 && ackPacket[3] == 0x00) {
			//softserial.println("NAK Received");
			return 1;
		}
	}

	for (i = 2; i < 8 ;i++) {
	CK_A = CK_A + ackPacket[i];
	CK_B = CK_B + CK_A;
	}
	if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
		//softserial.print("ACK Received");
		return 10;
				}
	else {
		//softserial.print("ACK Checksum Failure");
		return 1;
	}
}

void setBaud(byte baudSetting) {
	if (baudSetting == 0x12) Serial.begin(4800);
	if (baudSetting == 0x4B) Serial.begin(19200);
	if (baudSetting == 0x96) Serial.begin(38400);
	if (baudSetting == 0xE1) Serial.begin(57600);
	if (baudSetting == 0xC2) Serial.begin(115200);
	if (baudSetting == 0x84) Serial.begin(230400);
}
