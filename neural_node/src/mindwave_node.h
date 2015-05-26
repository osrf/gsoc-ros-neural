#ifndef _MINDWAVE_H_
#define _MINDWAVE_H_

#define MINDWAVE_BAUDRATE	57600

class Mindwave
{

public:
	void setup();
	void update();
	void setDebug(bool d) {debug = d;}
	void setTimeout(long t) {timeOut = t;} // in millis
	boolean hasNewData() {return newPacket;}
	boolean isDebugging() {return debug;}
	byte getAttention() {return attention;} // 0 (bad) to 100 (good)
	byte getMeditation() {return meditation;} // 0 (bad) to 100 (good)
	byte getPoorQuality() {return poorQuality;} // 0 (good) to 200 (bad)
	byte getQuality() {return 200 - poorQuality;} // 0 (bad) to 200 (good)
private:
	byte readOneByte();
	byte readFirstByte();
	boolean debug;
	boolean newPacket;
	byte payloadData[64];
	byte poorQuality;
	byte attention;
	byte meditation;

	long lastReceivedPacket;ls
	long timeOut;
};

#endif
