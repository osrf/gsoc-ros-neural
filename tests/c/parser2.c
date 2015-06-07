/* Parser types */
#define PARSER_TYPE_NULL 0x00
#define PARSER_TYPE_PACKETS 0x01 /* Stream bytes as ThinkGear Packets */
#define PARSER_TYPE_2BYTERAW 0x02 /* Stream bytes as 2-byte raw data */
/* Data CODE definitions */
#define PARSER_BATTERY_CODE 0x01
#define PARSER_POOR_SIGNAL_CODE 0x02
#define PARSER_ATTENTION_CODE 0x04
#define PARSER_MEDITATION_CODE 0x05
#define PARSER_RAW_CODE 0x80

THINKGEAR_initParser()
/**
* @param parser Pointer to a ThinkGearStreamParser object.
* @param parserType One of the PARSER_TYPE_* constants defined
* above: PARSER_TYPE_PACKETS or
* PARSER_TYPE_2BYTERAW.
* @param handleDataValueFunc A user-defined callback function that will
* be called whenever a data value is parsed
* from a Packet.
* @param customData A pointer to any arbitrary data that will
* also be passed to the handleDataValueFunc
* whenever a data value is parsed from a
* Packet.
*
* @return -1 if @c parser is NULL.
* @return -2 if @c parserType is invalid.
* @return 0 on success.
*/
int
THINKGEAR_initParser( ThinkGearStreamParser *parser, unsigned char parserType,
void (*handleDataValueFunc)(
unsigned char extendedCodeLevel,
unsigned char code, unsigned char numBytes,
const unsigned char *value, void *customData),
void *customData );

/**
* @param parser Pointer to an initialized ThinkGearDataParser object.
* @param byte The next byte of the data stream.
*
* @return -1 if @c parser is NULL.
* @return -2 if a complete Packet was received, but the checksum failed.
* @return 0 if the @c byte did not yet complete a Packet.
* @return 1 if a Packet was received and parsed successfully.
*
*/
int
THINKGEAR_parseByte( ThinkGearStreamParser *parser, unsigned char byte );

void handleDataValueFunc( unsigned char extendedCodeLevel, unsigned char code, 
					 unsigned char valueLength, const unsigned char *value, void *customData ) {
	
	if( extendedCodeLevel == 0 ) {
		switch( code ) {
			/* [CODE]: ATTENTION eSense */
			case( 0x04 ):
				printf( "Attention Level: %d\n", value[0] & 0xFF );
				break;
			/* [CODE]: MEDITATION eSense */
			case( 0x05 ):
				printf( "Meditation Level: %d\n", value[0] & 0xFF );
				break;
			/* Other [CODE]s */
			default:
				printf( "EXCODE level: %d CODE: 0x%02X vLength: %d\n", extendedCodeLevel, code, valueLength );
				printf( "Data value(s):" );
			
				for( i=0; i<valueLength; i++ )
					printf( " %02X", value[i] & 0xFF );
					printf( "\n" );
		}
	}
}

/**
* Program which reads ThinkGear Data Values from a COM port.
*/
int
main( int argc, char **argv ) {
/* 2) Initialize ThinkGear stream parser */
	ThinkGearStreamParser parser;
	THINKGEAR_initParser( &parser, PARSER_TYPE_PACKETS,handleDataValueFunc, NULL );
	/* TODO: Initialize 'stream' here to read from a serial data
	* stream, or whatever stream source is appropriate for your
	* application. See documentation for "Serial I/O" for your
	* platform for details.
	*/
	FILE *stream = fopen( "COM4", "r" );
	/* 3) Stuff each byte from the stream into the parser. Every time
	* a Data Value is received, handleDataValueFunc() is called.
	*/
	unsigned char streamByte;
	while( 1 ) {
		fread( &streamByte, 1, stream );
		THINKGEAR_parseByte( &parser, streamByte );
	}
}