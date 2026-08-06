function str4out = Int_To_4Str(input_int)
%Int_To_Str 将整型变量转换为4位字符串，小于1000自动往前补0
%   input_int - 整型或浮点型变量
%   str4out - 4位字符串
    str_temp = num2str(int16(input_int));
    
    if input_int < 1000
        str4out = [repmat('0',1,4 - length(str_temp)),str_temp];
    else
        str4out = str_temp;
    end

end