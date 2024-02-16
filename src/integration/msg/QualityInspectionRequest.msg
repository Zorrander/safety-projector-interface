##Description - request about a quality inspection for a particular productid and partid

## The unique id of this check
string check_id

## region of interest definition
int32 region_of_interest_x_offset
int32 region_of_interest_y_offset
int32 region_of_interest_height
int32 region_of_interest_width

## classification - inspection settings
string classifier_model
bool do_rectify