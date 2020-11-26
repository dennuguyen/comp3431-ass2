# ass2

## Resources

### Feature Matching
https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html

## Requirements

`pip3 install opencv-python`

<br />

## "Wall Follow" example

Run `roslaunch ass2_test test.launch`
This node publishes two topics
- /road_info : A uint32[] array that describes distances in the image.
    see road_info.png for what each index represents
- /lane/image/compressed : A mask representing the lane we're currently in.
    Works well in the simulator, but needs improvements with the real thing,
    since the dashed lines in the lab have much wider gaps

## Birds-eye view example

Run `ass2_test/test/test2.py` for a single image example
Read the comments for the logic behind it

