function [ LaserValues, LaserOrientation, LaserPosition ] = getLaserValues( string )
    %GETHOKUYOVALUES Summary of this function goes here
    %   Detailed explanation goes here
    C = strsplit(string,'\/','DelimiterType','RegularExpression');
    LaserValues=str2double(C(1:180));
    LaserOrientation = strsplit(C{181},'>','DelimiterType','RegularExpression');
    LaserPosition = strsplit(LaserOrientation{4},'<','DelimiterType','RegularExpression');
    LaserOrientation = radtodeg(str2double(LaserOrientation));
    LaserOrientation = LaserOrientation(1,1:3);
    LaserPosition = str2double(LaserPosition);
    LaserOrientation
    LaserPosition
end

