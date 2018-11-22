 clear all;close all;clc;
%% adjust this if you know the maximum array size in advance
  bufferSize = 1e5;
  data = zeros(bufferSize, 10);
%% use this for more flexibility
% DELIMITERS = '\t , ;'; % this is tab, space, comma, or semicolon as delimiter
numberOfGoodLinesSaved = 0;
total_lines = 0;
dir log
file_str=input("file?: ",'s'); %choose log file
filename = strcat('log\',file_str,'.txt'); % path of the log file you want to load (.txt)

fileID = fopen(filename, 'r');
testForNextLine = ~feof(fileID);
while (testForNextLine )
currentLine = fgetl(fileID);
  % parse current line
  currentLineNumerical = sscanf(currentLine, '%f');
  % use this line instead if you need the DELIMITER to vary or you want
  % more flexibility
%   currentLineNumerical = sscanf(currentLine, ['%f%*[' DELIMITERS ']');
  % check if current line is valid
  if ~isempty(currentLineNumerical) && numel(currentLineNumerical) == 10
      numberOfGoodLinesSaved = numberOfGoodLinesSaved + 1;
      data(numberOfGoodLinesSaved,:) = currentLineNumerical;
  else
      % do nothing
  end
  % check buffer size is big enough.  double the buffer size if needed
  if numberOfGoodLinesSaved == bufferSize
      bufferSize = 2*bufferSize;
      data(numberOfGoodLinesSaved+1:bufferSize,:) = 0;
  end
  testForNextLine = ~feof(fileID);
  total_lines = total_lines + 1;
end % end while loop
% delete extra lines in buffer
data(numberOfGoodLinesSaved+1:end,:) = [];
fclose(fileID);

fprintf('Finished Cleaning \n')
%% Write cleaned log on file
fid=fopen(strcat('log\',file_str,'_clean','.txt'),'w');
for i=1:length(data)
    fprintf(fid, '%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n', data(i,:));
end
fclose(fid);

fprintf('Cleaned log saved \n')