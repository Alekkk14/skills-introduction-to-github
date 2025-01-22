%% Configurazione onda
cfg = bluetoothWaveformConfig;
cfg.Mode = 'BR'; % onda Basic Rate
cfg.PacketType = 'FHS';
cfg.SamplesPerSymbol = 60;
cfg.WhitenInitialization = [0;0;0;0;0;1;1];

payloadLength = getPayloadLength(cfg); % Payload length in bytes
octetLength = 8;
dataBits = randi([0 1],payloadLength*octetLength,1); % Generate random payload bits
txWaveform = bluetoothWaveformGenerator(dataBits,cfg); % Create Bluetooth waveform

packetDuration = bluetoothPacketDuration(cfg.Mode,cfg.PacketType,payloadLength);

symbolRate = 1e6; % Symbol rate
sampleRate = symbolRate * cfg.SamplesPerSymbol;
numChannels = 10; % Number of channels
filterSpan = 8*any(strcmp(cfg.Mode,{'EDR2M','EDR3M'})); % To meet the spectral mask requirements

% Create and configure frequency offset System object
pfo = comm.PhaseFrequencyOffset('SampleRate',sampleRate);

% Create and configure spectrum analyzer System object
scope = spectrumAnalyzer('ViewType','Spectrum and spectrogram',...
    'TimeResolutionSource','property', ...
    'TimeResolution',1e-5,...
    'SampleRate',sampleRate, ...
    'TimeSpanSource','Property',...
    'TimeSpan',2e-3, ...
    'AxesLayout','Horizontal', ...
    'YLimits',[-70 25], ...
    'AveragingMethod','running');

% Loop over the number of channels to visaulize the frequency shift
for packetIdx = 1:numChannels
    channelNum = randsrc(1,1,0:60); % Generate random channel number
    freqIndex = channelNum - 39; % To visualize as a two sided spectrum
    pfo.FrequencyOffset = freqIndex*symbolRate; % Frequency shift
    hoppedWaveform = pfo(txWaveform(1:(packetDuration+filterSpan)*cfg.SamplesPerSymbol));
    scope.Title = ['Spectrum of Bluetooth ',cfg.Mode,...
        ' Waveform for Channel Number = ', num2str(channelNum)];
    scope(hoppedWaveform);
end
% Release the System objects
release(scope);

release(pfo);


% Initialize the parameters required for signal sink
txCenterFrequency = 2445000000;  % In Hz, varies between 2.402e9 to 2.480e9 with 1e6 spacing
txFrameLength     = length(txWaveform);
txNumberOfFrames  = 1e4;

% The default signal sink is 'File'
signalSink = 'PLUTO';

 % For 'ADALM-PLUTO'  
    connectedRadios = findPlutoRadio; % Discover ADALM-PLUTO radio(s) connected to your computer
    radioID = connectedRadios(1).RadioID;
    sigSink = sdrtx('Pluto',...
        'RadioID',           radioID,...
        'CenterFrequency',   txCenterFrequency,...
        'Gain',              0,...
        'SamplesPerFrame',   txFrameLength,...
        'BasebandSampleRate',sampleRate);
    % The transfer of baseband data to the SDR hardware is enclosed in a
    % try/catch block. This implies that if an error occurs during the
    % transmission, the hardware resources used by the SDR System object
    % are released.
    currentFrame = 1;
    try
        while currentFrame <= txNumberOfFrames
            % Data transmission
            sigSink(txWaveform);
            % Update the counter
            currentFrame = currentFrame + 1;
        end
    catch ME
        release(sigSink);
        rethrow(ME);
    end

% Release the signal sink
release(sigSink);