//
//  ofxOpenBciWifi.cpp
//
//  openFrameworks addOn to read and process data from the OpenBci Wifi module
//
//  Created by Sean Montgomery on 10/03/17.
//
//  This work is licensed under the MIT License
//

#include "ofxOpenBciWifi.h"

ofxOpenBciWifi::ofxOpenBciWifi(int samplingFreq)
{
	_tcpPort = 3000;
	_messageDelimiter = "\r\n";
	TCP.setMessageDelimiter(_messageDelimiter);
	TCP.setup(_tcpPort);

	_verboseOutput = true;
	_loggingEnabled = false;
	_nHeadsets = 0;

	_Fs = samplingFreq;
	_fftEnabled = true;
	_fftWindowSize = _Fs;
	_fftOverlap = _fftWindowSize / 2;
	_fftBuffersize = _fftWindowSize * 2;

	_fft = ofxFft::create(_fftWindowSize, OF_FFT_WINDOW_HAMMING);
	//int nBins = _fft->getBinFromFrequency(70, _Fs);

	_hpFiltEnabled = true;
	_hpFiltFreq = 1.f;
	_notchFiltEnabled = true;
	_notchFiltFreq = 60.f;
	_lpFiltEnabled = false;
	_lpFiltFreq = 50.f;
	_fftSmoothingEnabled = true;
	_fftSmoothingNwin = 7;
	_fftSmoothingNewDataWeight = 0.25f;
}

void ofxOpenBciWifi::setTcpPort(int port)
{
	// ToDo: add error checking
	_tcpPort = port;
	TCP.setup(_tcpPort);
}

int ofxOpenBciWifi::getTcpPort()
{
	return _tcpPort;
}

int ofxOpenBciWifi::getHeadsetCount()
{
	return _nHeadsets;
}

vector<string> ofxOpenBciWifi::getHeadsetIpAddresses()
{
	return _ipAddresses;
}

void ofxOpenBciWifi::enableDataLogging(string filePath)
{
	_logFileName = filePath;
	_logger.setDirPath("");
	_logger.setFilename(filePath);
	_loggingEnabled = true;
	_logger.startThread();
}

void ofxOpenBciWifi::disableDataLogging()
{
	_loggingEnabled = false;
	_logger.stopThread();
}

void ofxOpenBciWifi::update()
{
	clearDataVectors();

	for (unsigned int i = 0; i < (unsigned int)TCP.getLastID(); i++) 
	{
		int h = -1;		// Headset number
		if (!TCP.isClientConnected(i))continue;

		// get the ip and port of the client
		string port = ofToString(TCP.getClientPort(i));
		string ip = TCP.getClientIP(i);

		// Match the tcp client to the stored ipAddresses or add a new address
		for (int ipNum = 0; ipNum < _ipAddresses.size(); ipNum++)
		{
			if (_ipAddresses.at(ipNum).compare(ip) == 0)
			{
				h = ipNum;
			}
		}
		if (h == -1)
		{
			// we didn't find a match to the current ip, so add it
			h = _ipAddresses.size();
			addHeadset(ip);
		}

		// receive all the available messages, separated by _messageDelimiter
		string tmp;
		do {
			_stringData.at(h) = _stringData.at(h) + tmp;
			tmp = TCP.receive(i);
		} while (tmp != "");

		if (_verboseOutput)
		{
			ofLogVerbose("ofxOpenBciWifi") << _stringData.at(h);
		}

		// Split the string by "chunk"
		vector<string> result = ofSplitString(_stringData.at(h), "{\"chunk\":");
		for (int r = 1; r < result.size(); r++)
		{
			// Add back "chunk"
			result.at(r) = "{\"chunk\":" + result.at(r);
			bool success = json.parse(result.at(r));
			if (success)
			{
				int nSamples = 0;
				try {
					nSamples = json["chunk"].size();
				}
				catch (exception e) {}


				vector<int> writePosition; // Data write position (should only need single value instead of vector if everything works properly)
				if (nSamples > 0)
				{
					try {
						// Check the number of data channels
						int tmp = json["chunk"][0]["data"].size();
						if (tmp > _nChannels.at(h)) {
							// Number of channels has changed
							_nChannels.at(h) = tmp;

							// resize data vector channel size	 
							_data.at(h).resize(_nChannels.at(h));

							// Resize the fft vectors
							_fftBuffer.at(h).resize(_nChannels.at(h));
							_latestFft.at(h).resize(_nChannels.at(h));

							// Create filters for each channel
							_filterHP.at(h).resize(_nChannels.at(h));
							_filterNotch.at(h).resize(_nChannels.at(h));
							_filterLP.at(h).resize(_nChannels.at(h));

							for (int ch = 0; ch < _nChannels.at(h); ch++)
							{
								_fftBuffer.at(h).at(ch).resize(_fftBuffersize);
								_latestFft.at(h).at(ch).resize(_fftWindowSize / 2);

								// This will reset all filters when the number of channels changes
								_filterHP.at(h).at(ch) = ofxBiquadFilter1f(OFX_BIQUAD_TYPE_HIGHPASS, _hpFiltFreq / _Fs, 0.7071);
								_filterNotch.at(h).at(ch) = ofxBiquadFilter1f(OFX_BIQUAD_TYPE_NOTCH, _notchFiltFreq / _Fs, 0.7071);
								_filterLP.at(h).at(ch) = ofxBiquadFilter1f(OFX_BIQUAD_TYPE_LOWPASS, _lpFiltFreq / _Fs, 0.7071);
							}
						}
					}
					catch (exception e) 
					{
						bool debug = 1;
					}

					writePosition.resize(_nChannels.at(h));

					// resize data vector to fit new samples
					for (int ch = 0; ch < _nChannels.at(h); ch++)
					{
						writePosition.at(ch) = _data.at(h).at(ch).size();
						_data.at(h).at(ch).resize(writePosition.at(ch) + nSamples);
					}
				}

				for (int s = 0; s < nSamples; s++)
				{
					if (_loggingEnabled)
					{
						try {
							_logger.push(_ipAddresses.at(h) + ",");
							_logger.push(json["chunk"][s]["timestamp"].asString() + ",");
							_logger.push(json["chunk"][s]["sampleNumber"].asString() + ",");
						}
						catch (exception e) {}
					}

					for (int ch = 0; ch < _nChannels.at(h); ch++)
					{
						int wp = writePosition.at(ch) + s;
						try {
							_data.at(h).at(ch).at(wp) = json["chunk"][s]["data"][ch].asFloat();

							if (_hpFiltEnabled)
							{
								_data.at(h).at(ch).at(wp) = _filterHP.at(h).at(ch).update(_data.at(h).at(ch).at(wp));
							}
							if (_notchFiltEnabled)
							{
								_data.at(h).at(ch).at(wp) = _filterNotch.at(h).at(ch).update(_data.at(h).at(ch).at(wp));
							}
							if (_lpFiltEnabled)
							{
								_data.at(h).at(ch).at(wp) = _filterLP.at(h).at(ch).update(_data.at(h).at(ch).at(wp));
							}

							if (_loggingEnabled)
							{
								_logger.push(ofToString(_data.at(h).at(ch).at(wp)) + ",");
							}

							if (_fftEnabled)
							{
								// Fill up the FFT buffer
								_fftBuffer.at(h).at(ch).at(_fftWritePos.at(h)) = _data.at(h).at(ch).at(wp);
							}
						}
						catch (exception e) {
							bool debug = true;
						}
					}
					if (_loggingEnabled)
					{
						_logger.push("\n");
					}

					if (_fftEnabled)
					{
						_fftWritePos.at(h)++;
						//if (_fftWritePos >= _fftReadPos + _fftWindowSize)
						if (_fftWritePos.at(h) == _fftReadPos.at(h) + _fftWindowSize)
						{
							for (int ch = 0; ch < _nChannels.at(h); ch++)
							{
								// If the buffer is full, perform FFT
								vector<float> temp;
								temp.resize(_fftWindowSize);
								for (int n = 0; n < _fftWindowSize; n++)
								{
									temp.at(n) = _fftBuffer.at(h).at(ch).at(n + _fftReadPos.at(h));
								}
								_fft->setSignal(temp);
								float* curFft = _fft->getAmplitude();

								for (int n = 0; n < _fftWindowSize / 2; n++)
								{
									if (isfinite(_latestFft.at(h).at(ch).at(n)))
									{
										if (_fftSmoothingEnabled)
										{
											// Smooth the FFT over time so that after X windows only 20% "legacy" influence remains
											//float newDataWeight = 1.f - pow(10, log10(0.2) / _fftSmoothingNwin);
											// Calculate the FFT power in dB for easier viewing
											_latestFft.at(h).at(ch).at(n) = smooth(10.f * log10(curFft[n]), _latestFft.at(h).at(ch).at(n), _fftSmoothingNewDataWeight);
										}
										else
										{
											_latestFft.at(h).at(ch).at(n) = 10.f * log10(curFft[n]);
										}
									}
									else
									{
										// Handle case when fftData runs off into the weeds
										_latestFft.at(h).at(ch).at(n) = 10.f * log10(curFft[n]);
									}
								}
							}

							// Set fft buffer write position and read position
							if (_fftReadPos.at(h) + 2*_fftWindowSize - _fftOverlap - 1 <= _fftBuffersize)
							{
								_fftReadPos.at(h) = _fftReadPos.at(h) + _fftWindowSize - _fftOverlap;
							}
							else
							{
								for (int ch = 0; ch < _nChannels.at(h); ch++)
								{
									// FFT buffer is running out. Shift back to the beginning.
									vector<float> temp;
									temp.resize(_fftWindowSize - _fftOverlap);
									for (int n = 0; n < _fftWindowSize - _fftOverlap; n++)
									{
										temp.at(n) = _fftBuffer.at(h).at(ch).at(n + _fftReadPos.at(h) + _fftWindowSize - _fftOverlap);
									}
									for (int n = 0; n < _fftWindowSize - _fftOverlap; n++)
									{
										_fftBuffer.at(h).at(ch).at(n) = temp.at(n);
									}
									//copy(_fftBuffer.at(h).at(ch).begin() + _fftReadPos + _fftWindowSize - _fftOverlap,
									//	_fftBuffer.at(h).at(ch).begin() + _fftReadPos + _fftWindowSize - 1,
									//	_fftBuffer.at(h).at(ch).begin());
								}
								_fftReadPos.at(h) = 0;
								_fftWritePos.at(h) = _fftReadPos.at(h) + _fftWindowSize - _fftOverlap;
							}

							_newFftReady.at(h) = true;
						}
					}
				}
			}
		}
	}
}

float ofxOpenBciWifi::smooth(float newData, float oldData, float newDataWeight)
{
	return newData * newDataWeight + oldData * (1.f - newDataWeight);
}

void ofxOpenBciWifi::addHeadset(string ipAddress)
{
	_ipAddresses.push_back(ipAddress);
	int sz = _ipAddresses.size();
	_stringData.resize(sz);
	_data.resize(sz);
	_filterHP.resize(sz);
	_filterNotch.resize(sz);
	_filterLP.resize(sz);
	_latestFft.resize(sz);
	_fftBuffer.resize(sz);
	_nChannels.push_back(0);
	_newFftReady.push_back(false);
	_fftReadPos.push_back(0);
	_fftWritePos.push_back(0);
}

vector<string> ofxOpenBciWifi::getStringData()
{
	return _stringData;
}

string ofxOpenBciWifi::getStringData(string ipAddress)
{
	for (int h = 0; h < _ipAddresses.size(); h++)
	{
		if (ipAddress.compare(_ipAddresses.at(h)) == 0)
		{
			return _stringData.at(h);
		}
	}
}

vector<vector<float>> ofxOpenBciWifi::getData(string ipAddress)
{
	for (int h = 0; h < _ipAddresses.size(); h++)
	{
		if (ipAddress.compare(_ipAddresses.at(h)) == 0)
		{
			return _data.at(h);
		}
	}
}

vector<vector<float>> ofxOpenBciWifi::getLatestFft(string ipAddress)
{
	for (int h = 0; h < _ipAddresses.size(); h++)
	{
		if (ipAddress.compare(_ipAddresses.at(h)) == 0)
		{
			return _latestFft.at(h);
		}
	}
}

int ofxOpenBciWifi::getFftBinFromFrequency(float freq)
{
	return _fft->getBinFromFrequency(freq, _Fs);
}

void ofxOpenBciWifi::clearDataVectors()
{
	for (int h = 0; h < _data.size(); h++)
	{
		_stringData.at(h).clear();
		for (int ch = 0; ch < _data.at(h).size(); ch++)
		{
			_data.at(h).at(ch).clear();			// Headsets x Channels x Sample
			//_latestFft.at(h).at(ch).clear();		// Headsets x Channels x Frequency
		}
		_newFftReady.at(h) = false;
	}
}

bool ofxOpenBciWifi::isFftNew(string ipAddress)
{
	for (int h = 0; h < _ipAddresses.size(); h++)
	{
		if (ipAddress.compare(_ipAddresses.at(h)) == 0)
		{
			return _newFftReady.at(h);
		}
	}
}