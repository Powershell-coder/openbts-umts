/*
 * OpenBTS provides an open source alternative to legacy telco protocols and
 * traditionally complex, proprietary hardware systems.
 *
 * Copyright 2008, 2009 Free Software Foundation, Inc.
 * Copyright 2014 Range Networks, Inc.
 * Copyright 2014 Ettus Research LLC
 * 
 * This software is distributed under the terms of the GNU General Public 
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 * 
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#include "RadioInterface.h"
#include <Logger.h>

extern "C" {
#include "convert.h"
}

/*
 * Downlink digital gain scaling
 *
 * This value scales floating point transmit values prior fixed-point
 * conversion. The current value provides a significant amount of backoff to
 * avoid reaching compression across general device-daughterboard combinations.
 * More optimal values to target peak gain or maximum dynamic range will be
 * device and daughterboard combination specific.
 * */
#define TX_BASE_SCALING            150.0f

/*
 * Resampling ratio for 25 MHz base clocking
 *
 * This resampling factor absorbs the sample rate differences between UMTS chip
 * rate of 3.84 Mcps and the device clocking rate. USRP N200 clock operates at a
 * rate of 100 MHz with factor of 16 downsampling. B200 uses 25 MHz FPGA
 * clocking with downsampling by 4.
 */
#define RESAMP_INRATE              384
#define RESAMP_OUTRATE             384

/*
 * Number of taps per resampler-RRC filter partitions
 *
 * The number of taps per polyphase partition filter can be adjusted to suit
 * processing or spectrum requirements. Note that the filter length, and
 * associated group delay, affects timing.
 */
#define RESAMP_TAP_LEN             20

/* Chunk multiplier with resampler rates determine the chunk sizes */
#define CHUNK_MUL                  2

/* Universal resampling parameters */
#define NUMCHUNKS                  32

/* Receive scaling factor for 16 to 8 bits */
#define CONVERT_RX_SCALE           (128.0 / 32768.0)

/* Clipping detection threshold */
#define CLIP_THRESH                120.0f

UMTS::Time VectorQueue::nextTime() const
{
  UMTS::Time retVal;
  ScopedLock lock(mLock);

  while (!mQ.size())
    mWriteSignal.wait(mLock);

  return mQ.top()->time();
}

radioVector* VectorQueue::getStaleBurst(const UMTS::Time& targTime)
{
  ScopedLock lock(mLock);
  if (!mQ.size())
    return NULL;

  if (mQ.top()->time() < targTime) {
    radioVector* retVal = mQ.top();
    mQ.pop();
    return retVal;
  }
  return NULL;
}

radioVector* VectorQueue::getCurrentBurst(const UMTS::Time& targTime)
{
  ScopedLock lock(mLock);
  if (!mQ.size())
    return NULL;

  if (mQ.top()->time() == targTime) {
    radioVector* retVal = mQ.top();
    mQ.pop();
    return retVal;
  }
  return NULL;
}

RadioInterface::RadioInterface(RadioDevice *wRadio, int wReceiveOffset,
                               UMTS::Time wStartTime)
  : mRadio(wRadio), sendCursor(0), recvCursor(0),
    underrun(false), receiveOffset(wReceiveOffset),
    mOn(false), powerScaling(TX_BASE_SCALING),
    finalVec(NULL), upsampler(NULL), dnsampler(NULL),
    innerSendBuffer(NULL), innerRecvBuffer(NULL),
    outerSendBuffer(NULL), outerRecvBuffer(NULL),
    convertSendBuffer(NULL), convertRecvBuffer(NULL)
{
  mClock.set(wStartTime);
  inchunk = RESAMP_INRATE * CHUNK_MUL;
  outchunk = RESAMP_OUTRATE * CHUNK_MUL;
}

RadioInterface::~RadioInterface(void)
{
  delete convertSendBuffer;
  delete convertRecvBuffer;

  delete innerSendBuffer;
  delete outerSendBuffer;
  delete innerRecvBuffer;
  delete outerRecvBuffer;

  delete dnsampler;
  delete upsampler;
}

/*
 * Allocate resamplers and I/O buffers
 *
 * Setup multiple chunks on buffers except for the outer receive buffer, which
 * always receives a fixed number of samples. Conversion buffers are fixed point
 * used directly by the device interface.
 */
bool RadioInterface::init()
{
  innerRecvBuffer = new charVector(NUMCHUNKS * inchunk);

  /* Buffers that feed the resampler require filter length headroom */
  innerSendBuffer = new charVector(NUMCHUNKS * inchunk);

  return true;
}

/*
 * Start the radio device
 *
 * When the device start returns, initialize the timestamp counters with values
 * reported by the device. This avoids having to reset the device clock (if even
 * possible) and guessing at latency and corresponding initial values.
 *
 * Note that we do not advance the initial timestamp relative to receive here
 * so the initial data chunk will submit with a late timestamp. Some timing
 * compensation will occur with the TX_PACKET_SYNC alignment, which removes
 * packets from the downlink stream, while the remaining timing adjustment will
 * be absorbed by UHD and on the device itself.
 */
bool RadioInterface::start()
{
  if (mOn)
    return true;

  LOG(INFO) << "Starting radio device";

  if (!mRadio->start())
    return false;

  sendCursor = 0;
  recvCursor = 0;
  writeTimestamp = mRadio->initialWriteTimestamp();
  readTimestamp = mRadio->initialReadTimestamp();
  mOn = true;

  LOG(INFO) << "Radio started";
  return true;
}

/*
 * Stop the radio device
 *
 * This is a pass-through call to the device interface. Because the underlying
 * stop command issuance generally doesn't return confirmation on device status,
 * this call will only return false if the device is already stopped. 
 */
bool RadioInterface::stop()
{
  if (!mOn || !mRadio->stop())
    return false;

  mOn = false;
  return true;
}

double RadioInterface::fullScaleInputValue(void)
{
  return mRadio->fullScaleInputValue();
}

double RadioInterface::fullScaleOutputValue(void)
{
  return mRadio->fullScaleOutputValue();
}

int RadioInterface::setPowerAttenuation(int atten)
{
  double rfGain, digAtten;

  if (atten < 0)
    atten = 0;

  rfGain = mRadio->setTxGain(mRadio->maxTxGain() - (double) atten);
  digAtten = (double) atten - mRadio->maxTxGain() + rfGain;

  if (digAtten < 1.0)
    powerScaling = TX_BASE_SCALING;
  else
    powerScaling = TX_BASE_SCALING / sqrt(pow(10, (digAtten / 10.0)));

  return atten;
}

int RadioInterface::radioifyVector(charVector &in, char *out, bool zero)
{
  if (zero)
    memset(out, 0, in.size() * sizeof(Complex<char>));
  else
    memcpy(out, in.begin(), in.size() * sizeof(Complex<char>));

  return in.size();
}

void RadioInterface::unRadioifyVector(char *in, charVector& out)
{
  memcpy(out.begin(), in, out.size() * sizeof(Complex<char>));
}

bool RadioInterface::pushBuffer(void)
{
  if (sendCursor < inchunk)
    return true;
  if (sendCursor > innerSendBuffer->size()) {
    LOG(ALERT) << "Send buffer overflow";
  }

  mRadio->writeSamples((char *) innerSendBuffer->begin(), sendCursor,
                       &underrun, writeTimestamp);

  writeTimestamp += sendCursor;
  sendCursor = 0;

  return true;
}

static int detectClipping(float *buf, int len, float thresh)
{
	for (int i = 0; i < len; i++) {
		if (fabsf(buf[i]) > thresh)
			return 1;
	}

	return 0;
}

bool RadioInterface::pullBuffer()
{
  bool localUnderrun;

  if (recvCursor) {
    LOG(ALERT) << "recvCursor not zero";
  }

  size_t len = UMTS::gSlotLen;

  /* Outer buffer access size is fixed */
  size_t num_recv = mRadio->readSamples((char *) innerRecvBuffer->begin(),
                                        len,
                                        &overrun,
                                        readTimestamp,
                                        &localUnderrun);
  if (num_recv != len) {
    LOG(ALERT) << "Receive error " << num_recv;
    return false;
  }

  recvCursor = len;
  readTimestamp += len;

  return true;
}

bool RadioInterface::tuneTx(double freq)
{
  return mRadio->setTxFreq(freq);
}

bool RadioInterface::tuneRx(double freq)
{
  return mRadio->setRxFreq(freq);
}

void RadioInterface::driveTransmitRadio(radioVector &radioBurst,
                                        bool zeroBurst)
{
  if (!mOn)
    return;

  radioifyVector(radioBurst, (char *) innerSendBuffer->begin(), zeroBurst);

  sendCursor += radioBurst.size();
  pushBuffer();
}

void RadioInterface::driveReceiveRadio(int guardPeriod)
{
  if (!mOn)
    return;

  int vecSz = UMTS::gSlotLen + guardPeriod;

  pullBuffer();

  UMTS::Time recvClock = mClock.get();
  int recvSz = recvCursor;
  int readSz = 0;
  const int symbolsPerSlot = UMTS::gSlotLen;

  // while there's enough data in receive buffer, form received 
  //    UMTS bursts and pass up to Transceiver
  while (recvSz >= symbolsPerSlot) {
    UMTS::Time tmpTime = recvClock;
    mClock.incTN();
    recvClock.incTN();

    if (recvClock.FN() >= 0) {
      radioVector *rxBurst = new radioVector(vecSz, tmpTime);
      memcpy(rxBurst->begin(),
             innerRecvBuffer->begin() + readSz,
             rxBurst->size() * sizeof(Complex<char>));

      mReceiveFIFO.put(rxBurst);
    }

    readSz += symbolsPerSlot;
    recvSz -= symbolsPerSlot;
  }

  recvCursor = 0;

  if (!readSz)
    return;
}
