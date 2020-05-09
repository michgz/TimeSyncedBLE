#ifndef CONFIG_H__
#define CONFIG_H__

// Application-level configs


// Is this a "test" device? A test device doesn't return accelerometer data, it just returns
// a similar quantity of random bytes. The difference is that there is no need to wait for
// or temporally align the data it is always just available.
#define isTestDevice()   (false)

// Send consecutive values rather than random values
#define isTestDeviceDebug()   (false)



// In the relay (Central) case, instead of relaying the data it just sends a summary packet at
// the end.
#define isRelaySummary()   (true)



// For peripheral, flashes its LED when it receives a broadcast message.
// For central, flashes its LED when it sends a broadcast message.
#define isLedOnBroadcast()   (false)



// In the central case, upload results of each scan if connected upstream
#define isUploadScanResults()    (false)


// In the central case, transmit a time sync message at regular intervals. In the peripheral case,
// be sensitive to time sync messages.
#define isUseSyncTimer()     (true)

// Send a packet size 10000 bytes with increasing values to detect missing values.
#define isDebugPacket()  (false)

// On detecting a trigger, don't do anything other than inform the upstream
#define isUploadTriggerSize()   (true)

#endif // CONFIG_H__
