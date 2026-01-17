%{
This is more of a test program. Unsure if the pot data would be a good source of psuedo-aero data or not, and this was the way it was tested. 
As it turns out, the data was just noise. That being said, it should be a good test for future data sets.
%}

disp("loading table...")
T = readtable("9.30RearData.csv");
%%%---NOTE---



%disp(T(1,2));

numRows = height(T);

%for i = 1:numRows
%    if string(T{i, 3}) == "RearLeftPot" & string(T{i, 4}) ~= "0"
%        disp("Time Stamp:" +  string(T{i, 1}) + " Data:" + string(T{i, 4}) + " Unit:" + string(T{i, 5}));
%    end
%end
tempSpeed = -1;
tempPotr = -1;
tempPotl = -1;

tempTime = -1;

potAvg = -1;

count = 0;


saveTableRear = table([], [], 'VariableNames', {'Timestamp','Throttle'});

disp("Entering loop...")
for i = 1:numRows
    if string(T{i, 3}) == "ThrottlePedal"
        tempSpeed = T{i, 4}; 
        tempTime = T{i, 1};
    end
       

  if tempSpeed ~= -1 && tempTime ~= -1
      newRow = table(tempTime, tempSpeed, 'VariableNames', {'Timestamp','Throttle'});
      saveTableRear = [saveTableRear; newRow]; % Append newRow to saveTable

      tempSpeed = -1; % Reset tempSpeed for the next iteration
      tempPotl = -1; % Reset tempPotl for the next iteration
      tempPotr = -1; % Reset tempPotr for the next iteration
      tempTime = -1;
      count = count + 1;
      if mod(count,2500) == 0
        disp("adding value...  Count is at: " + count + "  Row: " + i)
      end
  end
      

end
writetable(saveTableRear, "SpeedToPotRear.csv")

T = readtable("SpeedtoPotRear.csv");

scatter(T.Speed, T.RearPotAvg, 'filled');
xlabel('Speed');
ylabel('Rear Pot Avg');
title('Scatter: Speed vs Rear Pot Avg');
grid on;




