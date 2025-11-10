clc; clear; close all;

% --- UART ---
port = "COM3";
baud = 115200;

s = serialport(port, baud);
configureTerminator(s, "CR/LF");  
flush(s);
s.Timeout = 60;                   

disp("Waiting...");


gotCal = false;
while ~gotCal
    if s.NumBytesAvailable > 0
        line = strtrim(readline(s));
        disp(line)
        if contains(upper(line), "CALIBRATION")
            disp(">> RECEIVED CALIBRATION");
            gotCal = true;
        end
    else
        pause(0.01);
    end
end

% teraz nasłuchujemy na prośby o masy i pokazujemy okienka
calDone = false;
while ~calDone
    if s.NumBytesAvailable > 0
        line = strtrim(readline(s));
        disp(line)

        if contains(line, "put MASS 1")s
            val = NaN;
            while ~isfinite(val)
                answ = inputdlg("Podaj MASA 1 [kg]:","Kalibracja",1,{"1.000"});
                if isempty(answ), continue; end
                val = str2double(answ{1});
            end
            writeline(s, sprintf("%.6f", val));  
        elseif contains(line, "Podaj MASA 2")
            val = NaN;
            while ~isfinite(val)
                answ = inputdlg("Podaj MASA 2 [kg]:","Kalibracja",1,{"2.000"});
                if isempty(answ), continue; end
                val = str2double(answ{1});
            end
            writeline(s, sprintf("%.6f", val));
        elseif contains(upper(line), "KALIBRACJA OK")
            disp(">> KALIBRACJA ZAKOŃCZONA");
            calDone = true;
        end
    else
        pause(0.01);
    end
end


D = [];                 
plik = "pomiary.csv";
nowy = ~isfile(plik);

if nowy
    fid = fopen(plik,"w");
    fprintf(fid,"nr_proby,obciazenie,rezystancja\n");
    fclose(fid);
end

disp("Czekam na START...");

while true
    line = strtrim(readline(s));
    disp(line)
    if strcmpi(line,"START")
        disp(">> DOSTAŁEM START");
        break;
    end
end

disp(">> Loguję 3 liczby w linijce: nr_proby obciazenie rezystancja");

while true
    line = strtrim(readline(s));      % oczekujemy 'a b c'
    vals = sscanf(line,"%f %f %f");

    if numel(vals)==3
        D(end+1,:) = vals.';
        disp(["OTRZYMAŁEM:", num2str(vals.')])

        fid = fopen(plik,"a");
        fprintf(fid, "%d,%.6f,%d\n", vals(1), vals(2), vals(3));
        fclose(fid);
    end

end
