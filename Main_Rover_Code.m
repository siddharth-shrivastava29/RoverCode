%File Locations - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%  J:\YaseminOA_2016\DynamixelSDK-master\matlab\protocol2.0
%  J:\YaseminOA_2016\DynamixelSDK-master\matlab\protocol2.0\roverCode
%D:\Dropbox\Research\Heterogeneous\Ground_Control\LabVIEW\Automation_NewSetup\SubVI\vision
%D:\Dropbox\Research\Heterogeneous\Ground_Control\LabVIEW\Automation_NewSetup\SubVI\other useful

% clc;
% clear all;

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
DEVICENAME                  = 'COM8';       % Port

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
%%
FRExtendUp = 762; %1
BLExtendUp = 762; %2
FLExtendUp = 562; %3
BRExtendUp = 562; %4
FRExtendDown = 612; %1
BLExtendDown = 612; %2
FLExtendDown = 712; %3
BRExtendDown = 712; %4

FRPropel = 256; %9
BLPropel = 768; %11
FLPropel = 768; %7
BRPropel = 256; %5
FRReset = 512; %9
BLReset = 512; %11
FLReset = 512; %7
BRReset = 512; %5
%%
% - - - - - - - - - - - - - - - MAIN SCRIPT - - - - - - - - - - - - - - -
% - - - - - - - - - - - - - - - MAIN SCRIPT - - - - - - - - - - - - - - -













%  wave_gait || loops = 4 || frequency = .6
%  rp_preferred || loops = 30 || frequency = .01

% 1 = wave_gait
% 2 = rp_preferred
cc = ''
dd = ''

gait = 2;

%Parameters
wheel_power = 300;              %Wheel Power
gait_delay = 8;                 %Delay between wheel on and gait execution
theta_initial = (-35)              %Initial Offset from [0,-90] sweep range. (Ex. 45 = [45, -45])
loops = 6;                      %Number of iterations of the gait pattern
frequency = .3;                 %Delay between iterating gait patterns
















%%
theta_0 = round(theta_initial*(2.8444444444444))
%%
if gait == 1
    %{
    FRExtendUp = 762; %1
    BLExtendUp = 762; %2
    FLExtendUp = 562; %3
    BRExtendUp = 562; %4
    FRExtendDown = 612; %1
    BLExtendDown = 612; %2
    FLExtendDown = 712; %3
    BRExtendDown = 712; %4

    FRPropel = 256; %9
    BLPropel = 768; %11
    FLPropel = 768; %7
    BRPropel = 256; %5
    FRReset = 512; %9
    BLReset = 512; %11
    FLReset = 512; %7
    BRReset = 512; %5
    %}
    FRExtendUp = 762; %1
    BLExtendUp = 612; %2
    FLExtendUp = 562; %3
    BRExtendUp = 712; %4
    FRExtendDown = 612; %1
    BLExtendDown = 612; %2
    FLExtendDown = 712; %3
    BRExtendDown = 712; %4

    FRPropel = 256; %9
    BLPropel = 768; %11
    FLPropel = 768; %7
    BRPropel = 256; %5
    FRReset = 512; %9
    BLReset = 512; %11
    FLReset = 512; %7
    BRReset = 512; %5

    %GAIT ARRAYS
    FLE = [ FLExtendDown FLExtendUp FLExtendUp FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown];
    FRE = [ FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendUp FRExtendUp FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown];
    BLE = [ BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendUp BLExtendUp BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown];
    BRE = [ BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendUp BRExtendUp BRExtendDown];
    FLR = [ FLReset FLReset FLPropel FLPropel FLReset FLReset FLReset FLReset FLReset FLReset FLReset FLReset FLReset FLReset FLReset FLReset];
    FRR = [ FRReset FRReset FRReset FRReset FRReset FRReset FRReset FRReset FRReset FRReset FRPropel FRPropel FRReset FRReset FRReset FRReset];
    BLR = [ BLReset BLReset BLReset BLReset BLReset BLReset BLPropel BLPropel BLReset BLReset BLReset BLReset BLReset BLReset BLReset BLReset];
    BRR = [ BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRReset BRPropel BRPropel];


    %BEGIN ROVER EXECUTION
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_GOAL_SPEED, 1024 + wheel_power);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_GOAL_SPEED, wheel_power);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_GOAL_SPEED, 1024 + wheel_power);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_GOAL_SPEED, wheel_power);
    pause(gait_delay);

    %1474
    for x=1: +1: loops
        z = 16;
        for i=1: +1: z

            %disp( [ 'i = ' num2str( i ) '/' num2str( z ) '  of ' num2str( x ) '/4' ' - - - - - '])
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRE(i));

            write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLE(i));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLE(i)); 
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRE(i));

            write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, (BRR(i) - theta_0));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, (FLR(i) + theta_0));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, (FRR(i) - theta_0));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, (BLR(i) + theta_0));
            pause(frequency);

        end
    end

    write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, BRReset)
    
    %Lift limbs up
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRExtendUp);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLExtendUp);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLExtendUp); 
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRExtendUp);
    %Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    pause(.2)

    % Close port
    closePort(port_num);

    % Unload Library
    unloadlibrary(lib_name);
end





%%
%RP PREFERRED GAIT------------------------------------------------------
if gait == 2
    FRExtendUp = 762; %1
    BLExtendUp = 762; %2
    FLExtendUp = 562; %3
    BRExtendUp = 562; %4
    FRExtendDown = 612; %1
    BLExtendDown = 612; %2
    FLExtendDown = 712; %3
    BRExtendDown = 712; %4

    FRPropel = 256; %9
    BLPropel = 768; %11
    FLPropel = 768; %7
    BRPropel = 256; %5
    FRReset = 512; %9
    BLReset = 512; %11
    FLReset = 512; %7
    BRReset = 512; %5

    %GAIT ARRAYS
    %{
    %RP Recommended 0 -> -90
    FLE = [ FLExtendUp FLExtendUp FLExtendUp FLExtendUp FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown]
    FRE = [ FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendUp FRExtendUp FRExtendUp FRExtendUp]
    BLE = [ BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendUp BLExtendUp BLExtendUp BLExtendUp BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown]
    BRE = [ BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendUp BRExtendUp BRExtendUp BRExtendUp BRExtendDown BRExtendDown BRExtendDown BRExtendDown]
    FLR = [ 704 640 576 FLReset 533 555 576 597 619 640 661 683 704 725 747 768]
    FRR = [ 491 469 448 427 405 384 363 341 320 299 277 256 320 384 448 FRReset]
    BLR = [ 704 725 747 768 704 640 576 BLReset 533 555 576 597 619 640 661 683]
    BRR = [ 405 384 363 341 320 299 277 256 320 384 448 BRReset 491 469 448 427]
%}
    
    %{
    %Diagonal 0 -> -90
    FLE = [ FLExtendDown FLExtendUp FLExtendUp FLExtendUp FLExtendUp FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown]
    FRE = [ FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendUp FRExtendUp FRExtendUp FRExtendUp FRExtendDown FRExtendDown FRExtendDown]
    BLE = [ BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendUp BLExtendUp BLExtendUp BLExtendUp BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown]
    BRE = [ BRExtendUp BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendUp BRExtendUp BRExtendUp]
    FLR = [ 768 704 640 576 FLReset 533 555 576 597 619 640 661 683 704 725 747]
    FRR = [ 427 405 384 363 341 320 299 277 256 320 384 448 FRReset 491 469 448]
    BLR = [ 683 704 725 747 768 704 640 576 BLReset 533 555 576 597 619 640 661]
    BRR = [ BRReset 491 469 448 427 405 384 363 341 320 299 277 256 320 384 448]
    %}
    
    %
    %RP Recommended +45 -> -45
    FRReset = 640; %9
    BLReset = 384; %11
    FLReset = 384; %7
    BRReset = 640; %5
    FLE = [ FLExtendUp FLExtendUp FLExtendUp FLExtendUp FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown FLExtendDown]
    FRE = [ FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendDown FRExtendUp FRExtendUp FRExtendUp FRExtendUp]
    BLE = [ BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendUp BLExtendUp BLExtendUp BLExtendUp BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown BLExtendDown]
    BRE = [ BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendDown BRExtendUp BRExtendUp BRExtendUp BRExtendUp BRExtendDown BRExtendDown BRExtendDown BRExtendDown]
    FLR = [ 576 512 448 FLReset 405 427 448 469 490 512 533 555 576 597 619 640]
    FRR = [ 619 597 576 555 533 512 490 469 448 427 405 384 448 512 576 FRReset]
    BLR = [ 576 597 619 640 576 512 448 BLReset 405 427 448 469 490 512 533 555]
    BRR = [ 533 512 490 469 448 427 405 384 448 512 576 BRReset 619 597 576 555]
%}
    %BEGIN ROVER EXECUTION
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_GOAL_SPEED, wheel_power);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_GOAL_SPEED, 1024 + wheel_power);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_GOAL_SPEED, wheel_power);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_GOAL_SPEED, 1024 + wheel_power);
    pause(gait_delay);

    %1474
    for x=1: +1: loops
        z = 16
        for i=1: +1: z

            disp( [ 'i = ' num2str( i ) '/' num2str( z ) '  of ' num2str( x ) '/4' ' - - - - - '])
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRE(i));

            write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLE(i));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLE(i)); 
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRE(i));

            write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, (BRR(i) - theta_0));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, (FLR(i) + theta_0));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, (FRR(i) - theta_0));
            write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, (BLR(i) + theta_0));
            pause(frequency);


        end
    end
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, 712);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, 712);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, 612);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, 612);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, BRReset)
    
    %Lift limbs up
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRExtendDown);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 5, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 7, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 9, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_MX_GOAL_POSITION, 512);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRExtendUp);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLExtendUp);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLExtendUp); 
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRExtendUp);
    %Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
    pause(.2)

    % Close port
    closePort(port_num);

    % Unload Library
    unloadlibrary(lib_name);
end
%%





















%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%Lift limbs up
write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, FRExtendUp);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_MX_GOAL_POSITION, BLExtendUp);
write2ByteTxRx(port_num, PROTOCOL_VERSION, 3, ADDR_MX_GOAL_POSITION, FLExtendUp); 
write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, BRExtendUp);
%Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, 10, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_MX_TORQUE_ENABLEWHEEL, TORQUE_DISABLE);
pause(.2)

% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);
