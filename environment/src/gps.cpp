#include "gps.h"
#include <QDebug>

#define ASCII_CHAR_OFFSET 	48
#define SIX_BIT_MASK 		0b00111111
#define FIVE_BIT_MASK 		0b00011111
#define FOUR_BIT_MASK 		0b00001111
#define TWO_BIT_MASK 		0b00000011
#define ONE_BIT_MASK 		0b00000001

int sign(double x)
{
	if (x > 0) return 1;
	if (x < 0) return -1;
	return 0;
}


navData::navData(uint32_t userID, double longitd, double latitd, double SOGmeterPrSec, double trackDeg)
{
	if (userID >= pow(2,30))
	{
		qDebug() << "Not a valid user ID:" << userID;
		return;
	}
	MMSI = userID;
	set_position(longitd, latitd);
	set_SOG(SOGmeterPrSec);
	set_track(trackDeg);
	status = UNDEFINED;
	ROT = -128;
	positionAccuracy = LOW;
	COG = 360;
	time = QTime::currentTime();
}



void navData::set_nav_status(navStatus ns)
{
	if( ns < 0 || ns > 15)
	{
		qDebug() << "Attempted to set invalid navigational status:" << ns;
		return;
	}
	status = ns;
}

void navData::set_ROT(double degPerSec )
{
	ROT = degPerSec;
}

void navData::set_SOG(double meterPrSec)
{
	if (meterPrSec < 0)
	{
		qDebug() << "Attempted to set invalid SOG:" << meterPrSec;
		return;
	}
	SOG = meterPrSec;
}

void navData::set_position_accuracy(posAccuracy pa)
{
	if (pa < 0 || pa > 1)
	{
		qDebug() << "Attempted to set invalid position accuracy:" << pa;
		return;
	}
	positionAccuracy = pa;
}

void navData::set_position(double longitd, double latitd)
{
	if (longitd < -180 || longitd > 180 || latitd < -90 || latitd > 90)
	{
		qDebug() << "Long/lat must be between +/- 180deg. Attempted to set:" << longitd << latitd;
		return;
	}
	longitude = longitd;
	latitude = latitd;
}

void navData::set_COG(double degrees)
{
	if (degrees >= 360 || degrees < 0)
	{
		qDebug() << "Attempted to set invalid COG:" << degrees;
		return;
	}
	COG = degrees;
}

void navData::set_track(double degrees)
{
	if (degrees >= 360 || degrees < 0)
	{
		qDebug() << "Attempted to set invalid track:" << degrees;
		return;
	}
	track = degrees;
}

void navData::set_time(QTime t)
{
	time = t;
}



int32_t decimal_degrees_to_AIS_minutes(double decimalDeg)
{

	int32_t AISminutes = 0;
	int8_t degrees = floor(decimalDeg);
	AISminutes += degrees*60*10000;
	double remainderMinutes = (decimalDeg - degrees)*60;
	AISminutes += remainderMinutes*10000;
	return AISminutes;
}


int32_t get_AIS_communication_state(int8_t syncState, int8_t slotTimeOut, QTime time)
{
	// syncState: 	2 bits
	// slotTimeOut: 3 bits
	if( syncState > 3 || syncState < 0 || slotTimeOut > 7 || slotTimeOut < 0)
	{
		qDebug() << "syncState og slotTimeOut out of range.";
		return 0;
	}
	int32_t commState = 0; // 19 bits
	commState |= (syncState << 17);
	commState |= (slotTimeOut << 14);

	int16_t subMessage = 0; // 14 bits

	if( slotTimeOut == 1 ) // sub message is UTC hour and minute
	{
		uint8_t hour = time.hour(); // 5 bits
		uint8_t minute = time.minute(); // 7 bits
		subMessage |= (hour << 9); // hour = bit 13 to 9 in subMessage
		subMessage |= (minute << 2); // minute = bit 8 to 2 in subMessage
	}
	else
	{
		qDebug() << "slotTimeOut != 1 not currently implemented.";
		return 0;
	}
	commState |= subMessage;
	return commState;


	// source: https://github.com/dma-ais/AisLib/blob/master/ais-lib-messages/src/main/java/dk/dma/ais/message/AisMessage1.java
}


string navData::get_AIS_class_A_position_report()
{
	uint8_t msgID = 1;
	uint8_t repeatIndicator = 0;
	int8_t aisROT = (int8_t) (4.733*sqrt(abs(ROT))*sign(ROT));
	uint16_t aisSOG = (uint16_t) (SOG*10);
	int32_t aisLongitude = decimal_degrees_to_AIS_minutes(longitude);
	int32_t aisLatitude = decimal_degrees_to_AIS_minutes(latitude);
	int16_t aisCOG = (int16_t) (COG*10);
	int16_t trueHeading = (int16_t) track;
	int8_t timeStamp = time.second();
	int8_t spare = 0;
	int8_t RAIM_flag = 0;
	int8_t syncState = 0;
	int8_t slotTimeOut = 1;
	int32_t commState = get_AIS_communication_state(syncState, slotTimeOut, time);

	const uint8_t payloadLength = 28;

	char AISpayload[payloadLength + 1] = {0};

	AISpayload[0] = msgID;
	AISpayload[1] = (repeatIndicator << 4) | (MMSI >> 26);
	AISpayload[2] = (MMSI >> 20) & SIX_BIT_MASK;
	AISpayload[3] = (MMSI >> 14) & SIX_BIT_MASK;
	AISpayload[4] = (MMSI >> 8) & SIX_BIT_MASK;
	AISpayload[5] = (MMSI >> 2) & SIX_BIT_MASK;
	AISpayload[6] = (MMSI << 4 & SIX_BIT_MASK) | (status & FOUR_BIT_MASK);
	AISpayload[7] = (aisROT >> 2) & SIX_BIT_MASK;
	AISpayload[8] = ((aisROT << 4) & SIX_BIT_MASK) | ((aisSOG >> 6) & FOUR_BIT_MASK);
	AISpayload[9] = aisSOG & SIX_BIT_MASK;
	AISpayload[10] = (positionAccuracy << 5) | ((aisLongitude >> 23) & FIVE_BIT_MASK);
	AISpayload[11] = (aisLongitude >> 17) & SIX_BIT_MASK;
	AISpayload[12] = (aisLongitude >> 11) & SIX_BIT_MASK;
	AISpayload[13] = (aisLongitude >> 5) & SIX_BIT_MASK;
	AISpayload[14] = ((aisLongitude << 1) & SIX_BIT_MASK) | ((aisLatitude >> 26) & ONE_BIT_MASK);
	AISpayload[15] = (aisLatitude >> 20) & SIX_BIT_MASK;
	AISpayload[16] = (aisLatitude >> 14) & SIX_BIT_MASK;
	AISpayload[17] = (aisLatitude >> 8) & SIX_BIT_MASK;
	AISpayload[18] = (aisLatitude >> 2) & SIX_BIT_MASK;
	AISpayload[19] = ((aisLatitude << 4) & SIX_BIT_MASK) | ((aisCOG >> 8) & FOUR_BIT_MASK);
	AISpayload[20] = (aisCOG >> 2) & SIX_BIT_MASK;
	AISpayload[21] = ((aisCOG << 4) & SIX_BIT_MASK) | ((trueHeading >> 5) & FOUR_BIT_MASK);
	AISpayload[22] = ((trueHeading << 1) & SIX_BIT_MASK) | ((timeStamp >> 5) & ONE_BIT_MASK);
	AISpayload[23] = ((timeStamp << 1) & SIX_BIT_MASK) | ((SMI >> 1) & ONE_BIT_MASK);
	AISpayload[24] = ((SMI << 5) & SIX_BIT_MASK) 
				| ((spare << 2) & FIVE_BIT_MASK) 
				| (( RAIM_flag << 1) & TWO_BIT_MASK) 
				| ((commState >> 18) & ONE_BIT_MASK);
	AISpayload[25] = (commState >> 12) & SIX_BIT_MASK;
	AISpayload[26] = (commState >> 6) & SIX_BIT_MASK;
	AISpayload[27] = commState & SIX_BIT_MASK;
	AISpayload[28] = '\0';

	for(int i = 0; i < payloadLength; i++)
	{
		if(AISpayload[i] > 39)
		{
			AISpayload[i] += 8;
		}
		AISpayload[i] += 48;
	}
	string AISmsg = "!AIVDM,1,1,,A,";
	AISmsg += AISpayload;
	AISmsg += ",0*7D"; // dummy checksum, will give error
	return AISmsg;
}


void navData::printData() const
{
	string s = "";
	s += "Time:\t\t\t";
	s += time.toString().toUtf8().constData();
	s += "\nLong:\t\t\t";
	s += to_string(longitude);
	s += "\nLat:\t\t\t";
	s += to_string(latitude);
	s += "\nHeading:\t\t";
	s += to_string(track);
	s += "\nROT:\t\t\t";
	s += to_string(ROT);
	s += "\nSOG:\t\t\t";
	s += to_string(SOG);
	s += "\nNavigational status:\t";
	switch(status) {
		case UNDERWAY_USING_ENGINE :
			s += "UNDERWAY_USING_ENGINE";
			break;
		case AT_ANCHOR :
			s += "AT_ANCHOR";
			break;
		case NOT_UNDER_COMMAND :
			s += "NOT_UNDER_COMMAND";
			break;
		case RESTRICTED_MANEUVERABILITY :
			s += "RESTRICTED_MANEUVERABILITY";
			break;
		case CONSTRAINED_BY_DRAUGHT :
			s += "CONSTRAINED_BY_DRAUGHT";
			break;
		case MOORED :
			s += "MOORED";
			break;
		case AGROUND :
			s += "AGROUND";
			break;
		case ENGAGED_IN_FISHING :
			s += "ENGAGED_IN_FISHING";
			break;
		case UNDER_WAY_SAILING :
			s += "UNDER_WAY_SAILING";
			break;
		case AIS_SART :
			s += "AIS_SART";
			break;
		case UNDEFINED :
			s += "UNDEFINED";
			break;
	}

	qDebug() << s.c_str();
}




gpsData::gpsData(double Longitude, double Latitude, double Heading, double HeadingRate, double Speed, double Altitude, QTime Time)
{
	longitude = Longitude;
	latitude = Latitude;
	heading = Heading;
	headingRate = HeadingRate;
	speed = Speed;
	altitude = Altitude;
	time = Time;
}

gpsData::gpsData()
{
	longitude = 0;
	latitude = 0;
	heading = 0;
	headingRate = 0;
	speed = 0;
	altitude = 0;
	time = QTime::currentTime();
}

void gpsData::print() const
{
	string s = "";
	s += "Time:\t\t";
	s += time.toString().toUtf8().constData();
	s += "\nLong:\t\t";
	s += to_string(longitude);
	s += "\nLat:\t\t";
	s += to_string(latitude);
	s += "\nHeading:\t";
	s += to_string(heading);
	s += "\nHdgRate:\t";
	s += to_string(headingRate);
	s += "\nSpeed:\t\t";
	s += to_string(speed);
	s += "\nAltitude:\t";
	s += to_string(altitude);
	qDebug() << s.c_str();
}