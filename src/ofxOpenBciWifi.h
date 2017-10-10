//
//  ofxOpenBciWifi.h
//
//  openFrameworks addOn to read and process data from the OpenBci Wifi module
//
//  Created by Sean Montgomery on 10/03/17.
//
//  This work is licensed under the MIT License
//

#pragma once

#include "ofxNetwork.h"
#include "ofxJSON.h"
#include "ofxBiquadFilter.h"
#include "ofxFft.h"
#include "ofxThreadedLogger.h"

class ofxOpenBciWifi
{
private:
	ofxTCPServer TCP;
	int _Fs;
	int _tcpPort;
	int _nHeadsets;
	string _messageDelimiter;						
	vector<string> _ipAddresses;
	vector<string> _stringData;
	vector<int> _nChannels;
	vector<vector<vector<float>>> _data;			// Headsets x Channels x Sample
	vector<vector<vector<float>>> _fftBuffer;		// Headsets x Channels x Sample
	vector<vector<vector<float>>> _latestFft;		// Headsets x Channels x Frequency
	
	ofxFft* _fft;
	int _fftWindowSize;					// Number of samples used to calculate fft. Default = Fs.
	int _fftBuffersize;
	int _fftOverlap;					// Number of overlapped samples between fft calculations. Default = fftWindowSize/2.
	bool _fftEnabled;
	vector<bool> _newFftReady;
	vector<int> _fftReadPos;
	vector<int> _fftWritePos;
	
	bool _hpFiltEnabled;
	float _hpFiltFreq;
	vector<vector<ofxBiquadFilter1f>> _filterHP;

	bool _notchFiltEnabled;
	float _notchFiltFreq;
	vector<vector<ofxBiquadFilter1f>> _filterNotch;

	bool _lpFiltEnabled;
	float _lpFiltFreq;
	vector<vector<ofxBiquadFilter1f>> _filterLP;

	bool _fftSmoothingEnabled;
	//int _fftSmoothingNwin;
	float _fftSmoothingNewDataWeight;

	bool _loggingEnabled;
	LoggerThread _logger;

	bool _verboseOutput;

	ofxJSONElement json;

	void addHeadset(string ipAddress);
	void clearDataVectors();

	vector<vector<int>> sample_numbers;

public:
	ofxOpenBciWifi(int samplingFreq = 250);
	void setTcpPort(int port);
	int getTcpPort();
	int getHeadsetCount();
	vector<string> getHeadsetIpAddresses();
	void enableDataLogging(string filePath);
	void disableDataLogging();
	void update();
	vector<string> getStringData();
	string getStringData(string ipAddress);
	vector<vector<float>> getData(string ipAddress);
	vector<vector<float>> getLatestFft(string ipAddress);
	int getFftBinFromFrequency(float freq);
	bool isFftNew(string ipAddress);

	// ** Planned functions **
	//void enableHPFilter(float freq);
	//void disableHPFilter();
	//void enableLPFilter(float freq);
	//void disableLPFilter();
	//void enableNotchFilter(float freq);
	//void disableNotchFilter();
	//void enableFftSmoothing(float newDataWeight = 0.25f);
	//void disableFftSmoothing();

	static float smooth(float newData, float oldData, float newDataWeight);
};