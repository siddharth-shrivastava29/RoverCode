clc;
clear all;
lib_name = '';
if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

% Control table address
ADDR_MX_TORQUE_ENABLEWHEEL  = 24;           % Control table address is different in Dynamixel model
ADDR_MX_GOAL_SPEED          = 32;
ADDR_MX_GOAL_POSITION       = 30;
ADDR_MX_PRESENT_POSITION    = 36;

PROTOCOL_VERSION            = 1.0;         
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM14';       % Port

TORQUE_ENABLE               = 0;            % Value for enabling the torque on AX12A
TORQUE_DISABLE              = 0;            % Value for disabling the torque on AX12A
DXL_MOVING_STATUS_THRESHOLD = 10;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop
COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

                                            % Initialize PortHandler Structs
                                            % Set the port path
                                            % Get methods and members of PortHandlerWindows
port_num = portHandler(DEVICENAME);         % Initialize PacketHandler Structs
packetHandler();
index = 1;
dxl_comm_result = COMM_TX_FAIL;             % Communication result
dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position
% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end
% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


% Enable Dynamixel Torque - - - - - - - - - - - - - - - - - - - - - - - - -
write1ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_ENABLE);
if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS          %dynamixel connection serial display
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
else
    fprintf('Dynamixel has been successfully connected \n');
    pause(2);
end

%-------------------------------------------------------------------------------------------------------------------------------------------------
%-------------------------------------------------------------------------------------------------------------------------------------------------


disp( [ 'FL {#n}'])
disp( [ 'FR {#n}'])
disp( [ 'BL {#n}'])
disp( [ 'BR {#n}'])

FRExtendUp = 712; %1
BLExtendUp = 712; %2
FLExtendUp = 612; %3
BRExtendUp = 612; %4
FRExtendDown = 612; %1
BLExtendDown = 612; %2
FLExtendDown = 712; %3
BRExtendDown = 712; %4

FRPropel = 552; %9
BLPropel = 312; %11
FLPropel = 472; %7
BRPropel = 712; %5
FRReset = 312; %9
BLReset = 512; %11
FLReset = 712; %7
BRReset = 512; %5

%GAIT ARRAYS
FLE = [ FLExtendDown FLExtendUp FLExtendUp FLExtendUp FLExtendUp FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown]
FRE = [ FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendUp FRExtendUp FRExtendUp FRExtendUp]
BLE = [ BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendUp BLExtendUp BLExtendUp BLExtendUp]
BRE = [ BRExtendDown BRExtendUp BRExtendUp BRExtendUp BRExtendUp BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown]
FLR = [ FLPropel FLPropel FLReset FLReset FLReset FLReset FLPropel FLPropel FLPropel FLPropel]
FRR = [ FRReset FRPropel FRPropel FRPropel FRPropel FRPropel FRPropel FRReset FRReset FRReset]
BLR = [ BLReset BLPropel BLPropel BLPropel BLPropel BLPropel BLPropel BLReset BLReset BLReset]
BRR = [ BRPropel BRPropel BRReset BRReset BRReset BRReset BRPropel BRPropel BRPropel BRPropel]

%BEGIN ROVER EXECUTION
write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, 712);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, 712);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, 612);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, 612);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, 512);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, 512);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, 512);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, 512);
pause(2);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_GOAL_SPEED, 1174);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_GOAL_SPEED, 150);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_GOAL_SPEED, 1174);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_GOAL_SPEED, 150);
pause(20);

for x=1: +1: 10
    z = 9
    for i=1: +1: z
       
        disp( [ 'i = ' num2str( i ) '/' num2str( z ) '  of ' num2str( x ) '/4' ' - - - - - '])
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRE(i));
        
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLE(i));
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLE(i)); 
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRE(i));
            
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, BRR(i));
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, FLR(i));
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, FRR(i));
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, BLR(i));
        pause(0.5);

    end
end

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);


% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;