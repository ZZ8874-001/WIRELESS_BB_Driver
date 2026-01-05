% serialportlist("all")
% serialportlist("available")
% clear;
clc;

if ~exist('user_usart','var')
    user_usart = serialport("COM7",9600);
end

user_usart.DataBits = 8;
user_usart.StopBits = 1;
user_usart.Parity = "none";
user_usart.Timeout = 10;

Tx_Buf = "1000 1000";

while true
    write(user_usart,Tx_Buf,"char");
    rx_buf = readline(user_usart);
    if strlength(rx_buf) == 9
        parts = strsplit(rx_buf,' ');
        buck_duty = str2double(parts{1})
        boost_duty = str2double(parts{2})
        
    end
    % clear receive_data;
end