import math

start_lng = 149.1652375
start_lat = -35.3632622
end_lng = 149.0042375
end_lat = -35.0132622

R = 6371000
start_lat = float(start_lat)
start_lng = float(start_lng)
end_lat = float(end_lat)
end_lng = float(end_lng)
# lat ==> b, long ==> a
start_x = (R * math.cos(start_lat) * math.cos(start_lng)) / 1000
start_y = (R * math.cos(start_lat) * math.sin(start_lng)) / 1000
# print("Received start location: ({0}, {1})".format(start_x, start_y))

end_x = (R * math.cos(float(end_lat)) * math.cos(float(end_lng))) / 1000
end_y = (R * math.cos(float(end_lat)) * math.sin(float(end_lng))) / 1000
# print("received end location: ({0}, {1})".format(end_x, end_y))