/**
  *
  * UBX_Parser.h - A header-only C++ class for parsing UBX messages from Ublox GPS
  *
  * Copyright (C) 2015 Simon D. Levy
  * Copyright (C) 2020 Christian Riggenbach
  *
  * This code is free software: you can redistribute it and/or modify
  * it under the terms of the GNU Lesser General Public License as
  * published by the Free Software Foundation, either version 3 of the
  * License, or (at your option) any later version.
  *
  * This code is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU Lesser General Public License
  * along with this code.  If not, see <http://www.gnu.org/licenses/>.
  */

#include <cstdint>

/**
  * A class for parsing UBX messages.
  */
class UBX_Parser {

  private:

    typedef enum {
      GOT_NONE,
      GOT_SYNC1,
      GOT_SYNC2,
      GOT_CLASS,
      GOT_ID,
      GOT_LENGTH1,
      GOT_LENGTH2,
      GOT_PAYLOAD,
      GOT_CHKA
    } state_t;

    state_t state;
    int msgclass;
    int msgid;
    int msglen;
    char chka;
    char chkb;
    int count;
    char* payload = nullptr;

    void addchk( int b ) {
      chka = ( chka + b ) & 0xFF;
      chkb = ( chkb + chka ) & 0xFF;
    }


    void dispatchMessage() {
      switch( msgid ) {
        // NAV_POSLLH
        case 0x02: {
          uint32_t iTOW = unpack_uint32( 0 );
          int32_t lon = unpack_int32( 4 );
          int32_t lat = unpack_int32( 8 );
          int32_t height = unpack_int32( 12 );
          int32_t hMSL = unpack_int32( 16 );
          uint32_t hAcc = unpack_uint32( 20 );
          uint32_t vAcc = unpack_uint32( 24 );
          handle_NAV_POSLLH( iTOW, lon, lat, height, hMSL, hAcc, vAcc );
        }
        break;

        // NAV_DOP
        case 0x04: {
          uint32_t iTOW = unpack_uint32( 0 );
          uint16_t gDOP = unpack_uint16( 4 );
          uint16_t pDOP = unpack_uint16( 6 );
          uint16_t tDOP = unpack_uint16( 8 );
          uint16_t vDOP = unpack_uint16( 10 );
          uint16_t hDOP = unpack_uint16( 12 );
          uint16_t nDOP = unpack_uint16( 14 );
          uint16_t eDOP = unpack_uint16( 16 );
          handle_NAV_DOP( iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP );
        }
        break;

        // NAV_PVT
        case 0x07: {
          uint32_t iTOW = unpack_uint32( 0 );
          uint8_t flags = unpack_uint8( 21 );
          bool rtkFix = flags & ( 1 << 2 );
          bool rtkFloat = flags & ( 1 << 1 );
          uint8_t numSatelites = unpack_uint8( 23 );
          double speed = double( unpack_int32( 60 ) ) * 1e-3;
          double headingOfMotion = double( unpack_int32( 64 ) ) * 1e-5;
          double headingOfVehicle = double( unpack_int32( 84 ) ) * 1e-5;
          double pDOP = double( unpack_int32( 76 ) ) * 1e-2;

          handle_NAV_PVT( iTOW, rtkFix, rtkFloat, numSatelites, speed, headingOfMotion, headingOfVehicle, pDOP );
        }
        break;

        // NAV_VELNED
        case 0x12: {
          uint32_t iTOW = unpack_uint32( 0 );
          int32_t velN = unpack_int32( 4 );
          int32_t velE = unpack_int32( 8 );
          int32_t velD = unpack_int32( 12 );
          uint32_t speed = unpack_uint32( 16 );
          uint32_t gSpeed = unpack_uint32( 20 );
          int32_t heading = unpack_int32( 24 );
          uint32_t sAcc = unpack_uint32( 28 );
          uint32_t cAcc = unpack_uint32( 32 );
          handle_NAV_VELNED( iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc );
        }
        break;

        // NAV_HPPOSLLH
        case 0x14: {
          uint32_t iTOW = unpack_uint32( 0 );
          double lon = double( unpack_int32( 8 ) ) * 1e-7;
          double lat = double( unpack_int32( 12 ) ) * 1e-7;
          double height = double( unpack_int32( 16 ) ) * 1e-3;
          double hMSL = double( unpack_int32( 20 ) ) * 1e-3;

          lon += double( unpack_int8( 24 ) ) * 1e-9;
          lat += double( unpack_int8( 25 ) ) * 1e-9;

          height += double( unpack_int8( 26 ) ) * 1e-4;
          hMSL += double( unpack_int8( 27 ) ) * 1e-4;

          double hAcc = double( unpack_uint32( 28 ) ) * 1e-4;
          double vAcc = double( unpack_uint32( 32 ) ) * 1e-4;
          handle_NAV_HPPOSLLH( iTOW, lon, lat, height, hMSL, hAcc, vAcc );
        }
        break;

        // NAV_RELPOSNED
        case 0x3C: {
          uint32_t iTOW = unpack_uint32( 4 );
          double relPosN = double( unpack_int32( 8 ) ) * 1e-2;
          double relPosE = double( unpack_int32( 12 ) ) * 1e-2;
          double relPosD = double( unpack_int32( 16 ) ) * 1e-2;
          double relPosLenght = double( unpack_int32( 20 ) ) * 1e-2;
          double relPosHeading = double( unpack_int32( 24 ) ) * 1e-2;

          relPosN += double( unpack_int8( 32 ) ) * 1e-4;
          relPosE += double( unpack_int8( 33 ) ) * 1e-4;
          relPosD += double( unpack_int8( 34 ) ) * 1e-4;
          relPosLenght += double( unpack_int8( 35 ) ) * 1e-4;

          handle_NAV_RELPOSNED( iTOW, relPosD, relPosE, relPosN, relPosLenght, relPosHeading );
        }
        break;

        default:
          reportUnhandled( msgid );
          break;
      }
    }

    uint32_t unpack_uint32( int offset ) {
      return ( uint32_t )unpack( offset, 4 );;
    }
    int32_t unpack_int32( int offset ) {
      return ( int32_t )unpack( offset, 4 );;
    }

    int16_t unpack_int16( int offset ) {
      return ( int16_t )unpack( offset, 2 );;
    }
    uint16_t unpack_uint16( int offset ) {
      return ( uint16_t )unpack( offset, 2 );;
    }

    int8_t unpack_int8( int offset ) {
      return ( int8_t )unpack( offset, 1 );;
    }
    uint8_t unpack_uint8( int offset ) {
      return ( uint8_t )unpack( offset, 1 );;
    }

    uint32_t unpack( int offset, int size ) {
      uint32_t value = 0; // four bytes on most Arduinos

      for( int k = 0; k < size; ++k ) {
        value <<= 8;
        value |= ( 0xFF & payload[offset + 4 - k - 1] );
      }

      return value;
    }

  protected:
    /**
      Override this method to handle NAV-POSLLH messages.
      @param iTOW GPS Millisecond Time of Week
      @param lon Longitude in degrees * 10<sup>7</sup>
      @param lat Latitude in degrees * 10<sup>7</sup>
      @param height Height above Ellipsoid in millimeters
      @param hMSL Height above mean sea level in millimeters
      @param hAcc Horizontal Accuracy Estimate in millimeters
      @param vAcc Vertical Accuracy Estimate in millimeters
      */
    virtual void handle_NAV_POSLLH(
            uint32_t /*iTOW*/,
            int32_t /*lon*/,
            int32_t /*lat*/,
            int32_t /*height*/,
            int32_t /*hMSL*/,
            uint32_t /*hAcc*/,
            uint32_t /*vAcc*/ ) { }

    /**
      Override this method to handle NAV-HPPOSLLH messages.
      @param iTOW GPS Millisecond Time of Week
      @param lon Longitude in degrees
      @param lat Latitude in degrees
      @param height Height above Ellipsoid in m
      @param hMSL Height above mean sea level in m
      @param hAcc Horizontal Accuracy Estimate in m
      @param vAcc Vertical Accuracy Estimate in m
      */
    virtual void handle_NAV_HPPOSLLH(
            uint32_t /*iTOW*/,
            double /*lon*/,
            double /*lat*/,
            double /*height*/,
            double /*hMSL*/,
            double /*hAcc*/,
            double /*vAcc*/ ) { }

    /**
      Override this method to handle NAV-DOP messages.
      @param iTOW GPS Millisecond Time of Week
      @param gDOP Geometric DOP
      @param pDOP Posiition DOP
      @param tDOP Time DOP
      @param vDOP Vertical DOP
      @param hDOP Horizontal DOP
      @param nDOP Northing DOP
      @param eDOP Easting DOP
      */
    virtual void handle_NAV_DOP(
            uint32_t /*iTOW*/,
            uint16_t /*gDOP*/,
            uint16_t /*pDOP*/,
            uint16_t /*tDOP*/,
            uint16_t /*vDOP*/,
            uint16_t /*hDOP*/,
            uint16_t /*nDOP*/,
            uint16_t /*eDOP*/ ) { }

    /**
      Override this method to handle NAV-VELNED messages.
      @param iTOW GPS Millisecond Time of Week
      @param velN NED north velocity in cm/sec
      @param velE NED east velocity in cm/sec
      @param velD NED down velocity in cm/sec
      @param speed Speed (3-D)in cm/sec
      @param gSpeed Ground Speed (3-D)in cm/sec
      @param heading Heading of motion 2-D in degrees * 10<sup>5</sup>
      @param sAcc Speed Accuracy Estimate in cm/sec
      @param cAcc Course / Heading Accuracy Estimate in degrees
      */
    virtual void handle_NAV_VELNED(
            uint32_t /*iTOW*/,
            int32_t /*velN*/,
            int32_t /*velE*/,
            int32_t /*velD*/,
            uint32_t /*speed*/,
            uint32_t /*gSpeed*/,
            int32_t /*heading*/,
            uint32_t /*sAcc*/,
            uint32_t /*cAcc*/ ) { }

    /**
      Override this method to handle NAV-POSLLH messages.
      @param iTOW GPS Millisecond Time of Week
      @param relPos Relative position to base, N component of vector in m
      @param relPos Relative position to base, E component of vector in m
      @param relPos Relative position to base, D component of vector in m
      @param relPos Relative position to base, Length component of vector in m
      @param relPos Relative position to base, Heading component of vector in degrees
      */
    virtual void handle_NAV_RELPOSNED(
            uint32_t /*iTOW*/,
            double /*relPosN*/,
            double /*relPosE*/,
            double /*relPosD*/,
            double /*relPosLenght*/,
            double /*relPosHeading*/ ) { }

    /**
      Override this method to handle NAV-PVT messages.
      @param iTOW GPS Millisecond Time of Week
      @param rtkFix true if Fixed RTK
      @param rtkFloat true if Float RTK
      @param numSatelites number of Satelites
      @param speed Speed in m/s
      @param headingOfMotion heading reported from the receiver in deg
      @param headingOfVehicle heading reported from the receiver in deg
      @param hAcc horizontal Accuracy in m
      @param vAcc vertical Accuracy in m
      @param pDOP position Dilution of Precision
      */
    virtual void handle_NAV_PVT(
            uint32_t /*iTOW*/,
            bool /*rtkFix*/,
            bool /*rtkFloat*/,
            uint8_t /*numSatelites*/,
            double /*speed*/,
            double /*headingOfMotion*/,
            double /*headingOfVehicle*/,
            double /*pDOP*/ ) { }



    /**
      * Override this method to report receipt of messages not
      * handled by current code.
      * @param msgid ID of current message
      */
    virtual void reportUnhandled( char /*msgid*/ ) { }

  public:

    /**
      * Constructs a UBX parser.
      */
    UBX_Parser() {
      payload = new char[1024];
      state    = GOT_NONE;
      msgclass = -1;
      msgid    = -1;
      msglen   = -1;
      chka     = -1;
      chkb     = -1;
      count    = 0;
    }

    ~UBX_Parser() {
      delete payload;
    }

    /**
      * Parses a new byte from the GPS. Automatically calls handle_ methods when a new
      * message is successfully parsed.
      * @param b the byte
      */
    void parse( const uint8_t b ) {
      if( b == 0xB5 ) {
        state = GOT_SYNC1;
      }

      else if( b == 0x62 && state == GOT_SYNC1 ) {
        state = GOT_SYNC2;
        chka = 0;
        chkb = 0;
      }

      else if( state == GOT_SYNC2 ) {
        state = GOT_CLASS;
        msgclass = b;
        addchk( b );
      }

      else if( state == GOT_CLASS ) {
        state = GOT_ID;
        msgid = b;
        addchk( b );
      }

      else if( state == GOT_ID ) {
        state = GOT_LENGTH1;
        msglen = b;
        addchk( b );
      }

      else if( state == GOT_LENGTH1 ) {
        state = GOT_LENGTH2;
        msglen += ( b << 8 );
        count = 0;
        addchk( b );
      }

      else if( state == GOT_LENGTH2 ) {
        addchk( b );
        payload[count] = b;
        count += 1;

        if( count == msglen ) {

          state = GOT_PAYLOAD;
        }
      }

      else if( state == GOT_PAYLOAD ) {
        state = ( b == chka ) ? GOT_CHKA : GOT_NONE;
      }

      else if( state == GOT_CHKA ) {
        if( b == chkb ) {
          dispatchMessage();
        }

        else {
          state = GOT_NONE;
        }
      }
    }
};
