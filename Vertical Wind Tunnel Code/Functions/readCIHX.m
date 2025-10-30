function [CIHX_info] = readCIHX(filename)
%This function reads CIHX files (NOT .CIH!!) and returns metadata such as
%resolution, framerate, exposuretime, bitdepth, startframe, framenumber,
%all returned in struct 'CIHX_info'. 
%Feel free to add more lines, as more there is more info available in the 
%cihx file. 

%Made by Jelle Schoppink j.j.schoppink@utwente.nl
%Version 1: January 2021

%If filetype .cihx is not included, add it.
if length(filename)<5 || sum(filename(end-4:end) == '.cihx')<5
    filename = [filename '.cihx'];
end

% fid1=fopen(sprintf('%s.cihx',filename),'r');
fid1=fopen(filename);
if fid1 < 1
    disp([filename ' filenames could not be found']);
    CIHX_info=0;
else
    %Read the CIHX file.
    Header=textscan(fid1,'%s','delimiter',':');
%     if contains(filename,'\') || contains(filename,'/')
%         dummy = find(filename == '\' | filename == '/' ,1,'last');
%         filename = filename(dummy+1:end);
%     end
        
    CIHX_info.Filename = filename(1:end-5); %Add the filename
    CIHX_info.Header = Header{1}; %And the whole header, just in case.
    CIHX_info.BitOrder = 'n'; %Initiate BitOrder
    for i = 1:length(Header{1})
        %Now read all header lines, and find the specific values. 
        %These header lines are not always in the same row, for which
        %reason they have to be 'found' each time again.
        thisLine = Header{1}{i};
        if ~isfield(CIHX_info,'fileType') && contains(thisLine,'fileFormat')
            type_begin = regexp(thisLine,'<');
            type_end = regexp(thisLine,'>');
            CIHX_info.fileType = thisLine(type_end(1)+1:type_begin(2)-1);
        elseif ~isfield(CIHX_info,'digitsFileNo') && contains(thisLine,'digitsOfFileNumber')
            CIHX_info.digitsFileNo = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'Width') && contains(thisLine,'width')
            CIHX_info.Width = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'Height') && contains(thisLine,'height')
            CIHX_info.Height = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'TotalFrames') && contains(thisLine,'totalFrame')
            CIHX_info.TotalFrames = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'StartFrame') && contains(thisLine,'startFrame')
            CIHX_info.StartFrame = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'FrameRate') && contains(thisLine,'recordRate')
            CIHX_info.FrameRate = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'SkipFrame') && contains(thisLine,'skipFrame')
            CIHX_info.SkipFrame = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'ShutterRate') && contains(thisLine,'shutterSpeed')
            CIHX_info.ShutterRate = str2double(thisLine(regexp(thisLine,'\d')));
        elseif ~isfield(CIHX_info,'BitNo') && contains(thisLine,'depth')
            CIHX_info.BitNo = str2double(thisLine(regexp(thisLine,'\d')));
        elseif CIHX_info.BitOrder == 'n' && contains(thisLine,'side')
            if contains(thisLine,'Lower')
                CIHX_info.BitOrder = 'b';
            elseif contains(thisLine,'Higher')
                CIHX_info.BitOrder = 'l';
            end
        elseif ~isfield(CIHX_info,'Color') && contains(thisLine,'type')
            if contains(thisLine,'Mono') | contains(thisLine,'Raw')
                CIHX_info.Color = 1;
                CIHX_info.ColorType = 'Mono';
            elseif contains(thisLine,'Color')
                CIHX_info.Color = 3;
                CIHX_info.ColorType = 'RGB';
            elseif contains(thisLine,'RawBayer')
                CIHX_info.Color = 3;
                CIHX_info.ColorType = 'Bayer';
            end
        elseif ~isfield(CIHX_info,'Date') && contains(thisLine,'date')
            date_begin = regexp(thisLine,'<');
            date_end = regexp(thisLine,'>');
            full_date = thisLine(date_end(1)+1:date_begin(2)-1);
            try
                dateslash = find(full_date == '/');
                year = str2double(full_date(1:dateslash(1)-1));
                month = str2double(full_date(dateslash(1)+1:dateslash(2)-1));
                day = str2double(full_date(dateslash(2)+1:end));
                hr_string = Header{1}{i+1};
                hr_begin = find(hr_string == '>');
                hr = str2double(hr_string(hr_begin+1:end));
                minute = str2double(Header{1}{i+2});
                sec_string = Header{1}{i+3};
                sec_end = find(sec_string == '<');
                sec = str2double(Header{1}{i+3}(1:sec_end-1));
                CIHX_info.Date = datetime(year,month,day,hr,minute,sec);
            catch
                CIHX_info.Date = full_date;
            end
        elseif ~isfield(CIHX_info,'CameraName') && contains(thisLine,'deviceName')
            name_begin = regexp(thisLine,'<');
            name_end = regexp(thisLine,'>');
            CIHX_info.CameraName = thisLine(name_end(1)+1:name_begin(2)-1);
        end
    end
end
end