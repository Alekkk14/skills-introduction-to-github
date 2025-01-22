%% Parametri canale fisico e simulazione
environment = 'outdoor'; % ambiente esterno: ho solo cammino a 2 raggi 
% posso scegliere tra: outdoor, industrial, home, office
distance = 5; % decido una distanza arbitraria tra tx e rx
EbNo = 25; % decido Eb/No in dB

% per simulazione in tempi brevi:
%maxNumErrs = 100; % numero massimo di errore di bit
%maxNumBits = 1e6; % numero massimo di bit
maxNumPkts = 1000; % numero massimo di pacchetti
%% Parametri per creazione onda BT
phyMode = 'BR'; % trasmissione in Basic Rate
bluetoothPacket = 'DH5'; % tipo di pacchetto
sps = 8; % samples per symbol

%% Configurazione disturbi RF
frequencyOffset = 6000; % in Hz
timingOffset = 0.5; % offset del simbolo, in campioni
timingDrift = 2; % in parti per milione
dcOffset = 2; % percentuale rispetto al valore di ampiezza massima

%% Generazione onda BT (Tx)
txCfg = bluetoothWaveformConfig('Mode', 'BR', ...
    'PacketType', 'DH5', ...  % ! non prende in automatico la lungh. del payload di alcuni type   
    'PayloadLength', 339, ...  % 2712 bit= 339 char (l'ho dovuto scrivere a mano)
    'SamplesPerSymbol', sps); % configura onda

packetDuration = bluetoothPacketDuration(phyMode, bluetoothPacket, dataLen);
filterSpan = 8*any(strcmp(phyMode,{'EDR2M', 'EDR3M'}));
packetDurationSpan = packetDuration + filterSpan;

 dataLen = getPayloadLength(txCfg); %lunghezza payload

 bitsPerByte = 8; % 1B=8bits


 errorCalc= comm.ErrorRate;
 berVec = zeros(3,1);
 loopCount = 0;
 pktCount = 0;
 pktErr = 0;

fid = fopen('testo.txt', 'r+');
message = fscanf(fid, '%c', [1, inf]);
fclose(fid);
lunghMess = strlength(message);
L = lunghMess;
if (L ~= dataLen)
    while (L < (dataLen*8))
        fopen('testo.txt', 'a');
        fprintf(fid, '0');
        L = L +1;
        fclose(fid);
    end
end

fopen('testo.txt', 'r');
message = fscanf(fid, '%c', [1, inf]);
txBits = reshape(dec2bin(message, 8).'-'0', 1, []).';
fclose(fid);

% while (loopCount < maxNumPkts)
    txWaveform = bluetoothWaveformGenerator(txBits, txCfg); % genera onda
    %% Aggiunta distorsioni all'onda
    timingDelayObj = dsp.VariableFractionalDelay; % crea oggetto 'timing offset'
 
    symbolRate = 1e6; % in Hz
    frequencyDelay = comm.PhaseFrequencyOffset('SampleRate', symbolRate*sps); %crea oggetto 'frequency offset'

    % aggiunta di frequency offset ad onda
    frequencyDelay.FrequencyOffset = frequencyOffset;
    txWaveformCFO = frequencyDelay(txWaveform); 

    % aggiunta di timing delay all'onda
    packetDuration = bluetoothPacketDuration(phyMode, bluetoothPacket, dataLen);
    filterSpan = 8*any(strcmp(phyMode,{'EDR2M', 'EDR3M'}));
    packetDurationSpan = packetDuration + filterSpan;
    totalTimingDrift = zeros(length(txWaveform), 1);
    timingDriftRate = (timingDrift*1e-6)/(packetDurationSpan*sps);
    timingDriftVal = timingDriftRate*(0:1:((packetDurationSpan*sps))-1)'; % timing drift
    totalTimingDrift(1:(packetDurationSpan*sps)) = timingDriftVal;
    timingDelay = (timingOffset*sps) + totalTimingDrift; % timing offset e timing drift statici
    txWaveformTimingCFO = timingDelayObj(txWaveformCFO, timingDelay); %aggiunta timing delay
    
    % aggiunta di DC offset all'onda
    dcValue = (dcOffset/100)*max(txWaveformTimingCFO);
    txImpairedWaveform = txWaveformTimingCFO + dcValue;
    
    %% Attenuazione dovuta al percorso
    [plLinear, pldB] = helperBluetoothEstimatePathLoss(environment, distance); % ottengo pl in dB
    
    txAttenWaveform = txImpairedWaveform./plLinear; % attenuazione dell'onda con path loss
    
    %% Aggiunta di AWGN all'onda
    codeRate = 2/3; % ATTENZIONE: dipende dal tipo di pacchetto 
    
    % if any(strcmp(bluetoothPacket,{'FHS','DM1','DM3','DM5','HV2','DV','EV4'})
    %    codeRate = 2/3;
    % elseif strcmp(bluetoothPacket,'HV1')
    %    codeRate = 1/3;
    % else
    %    codeRate = 1;
    % end
    
    % scegli bit per simbolo in base a tipo di trasmissione
    bitsPerSymbol = 1 + (strcmp(phyMode, 'EDR2M'))*1 + (strcmp(phyMode, 'EDR3M'))*2;
    
    snr = EbNo + 10*log10(codeRate) + 10*log10(bitsPerSymbol) - 10*log10(sps); % SNR del rumore AWGN
    rxWaveform = awgn(txAttenWaveform, snr, 'measured'); % aggiunta di AWGN --> ho ottenuto l'onda finale che capto al Rx
    
    %% Configurazione Rx
    rxCfg = getPhyConfigProperties(txCfg); % copia configurazione onda da Tx
    [rxBits, decodedInfo, pktStatus] = helperBluetoothPracticalReceiver(rxWaveform, rxCfg); % creazione di Rx
    numOfSignals = length(pktStatus);
   % pktCount =pktCount + numOfSignals;
   % loopCount = loopCount + 1;

    messageRx = reshape(char(bin2dec(reshape(char(rxBits+'0'), 8, []).')), 1, []);
    messageRx = messageRx(1:end-((dataLen*8)-lunghMess));
    fid = fopen('testoRx.txt' , 'w');
    fprintf(fid, '%c', messageRx);
    fclose(fid);

    
    % calcoli per BER e PER
    L1 = length(txBits);
    L2 = length(rxBits);
    L = min(L1, L2);
    if(isempty(L))
        berVec = errorCalc(txBits(1:L), rxBits(1:L));
    end
    pktErr = pktErr + sum(pktStatus); 
% end

%% Visualizzo onda trasmessa e ricevuta
specAnalyzer = spectrumAnalyzer( ...
    'ViewType','Spectrum and spectrogram', ...
    'Method','welch', ...
    'NumInputPorts',2, ...
    'AveragingMethod','exponential',...
    'SampleRate',symbolRate*sps,...
    'Title','Spectrum of Transmitted and Received Bluetooth BR/EDR Signals',...
    'ShowLegend',true, ...
    'ChannelNames',{'Transmitted Bluetooth BR/EDR signal','Received Bluetooth BR/EDR signal'});
specAnalyzer(txWaveform(1:packetDurationSpan*sps),rxWaveform(1:packetDurationSpan*sps));
release(specAnalyzer);

%% Calcolo BER (Bit Error Rate) e PER (Packet Error Rate)
per = pktErr/pktCount;
perDisplay = num2str(per);
ber = berVec(1);
if ((ber== 0) && (per == 1))
    ber = 0.5; % If packet error rate is 1, consider average BER of 0.5
end
if ((~any(strcmp(bluetoothPacket,{'ID','NULL','POLL'})))&&((length(txBits) == length(rxBits))))
    ber = (sum(xor(txBits,rxBits))/length(txBits));
    berDisplay = num2str(ber);
else % BER non può essere calcolato se il pacchetto è perso o è del tipo ID, NULL, POLL 
    berDisplay = 'Not applicable';
end

%% Mostra risultati a schermo
disp(['Input configuration: ', newline , '    PHY transmission mode: ', phyMode,....
    newline,'    Environment: ', environment, newline ,...
    '    Distance between the transmitter and receiver: ', num2str(distance), ' m', newline ,...
    '    Eb/No: ', num2str(EbNo), ' dB']);

disp(['Estimated outputs: ', newline , '    Path loss : ', num2str(pldB), ' dB'....
    newline, '    BER: ', berDisplay, '    PER: ', perDisplay]);

disp (['   N. Pacchetti: ', pktCount]);

