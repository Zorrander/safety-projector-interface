##Description - report about an error in quality inspection

## the product that is being inspected
string productId

## When the check took place
time timestamp

## The part id that is related to the defect
string partId

## A related image for reference
sensor_msgs/CompressedImage resource_image

##How certain the sender is that there is a defect from 0(not at all certain) to 100 (completely certain)
float32 certainty

##True means Quality Is OK
bool is_ok