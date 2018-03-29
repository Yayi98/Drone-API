## This package is modified Whycon package [original](https://github.com/lrse/whycon)
This package is used to change the targets value in the run time

### How to use this package
1. Take a backup of your current whycon package and remove it from the workspace.
2. Clone this [package](https://github.com/simmubhangu/whycon) in your workspace and build you workspace
3. Launch the whycon.launch file or you can use the same file, you were using previously to detect the marker

### How to change the targets in the runtime
1. This package use whycon/reset service to change the target value.
2. This reset service has 3 arguments:
    > Rosservice name: /whycon/reset

    > Requests: number, threshold

    > Response: targets
  *  number: number is the value of target. if you want to change the target value to 2 then value of number should be 2.
  *  threshold: In case you want to apply some condtion on the basic of marker, you can send the value and modify the code.
  * targets: this is the responce of the service which gives the current targets value on which algo is working. in case you need the value of target, you can use this value.
3. Use the following command in another terminal to change the target value
    ```sh
    rosservice call /whycon/reset "number: 2
    threshold: 0"
    ```
  The above command will change the target value to 2
  
4. lets take a look in the modified code.

  ```cpp
    bool whycon::WhyConROS::reset(whycon::SetNumberOfTargets::Request& request, whycon::SetNumberOfTargets::Response& response)
    {
      should_reset = true;
      targets = (int)request.number;
      marker_tresh = request.threshold;
      response.targets = targets;

      system = 0;
      return true;
    }
  ```
### How to use rosservice in the python
* import the service in the code 
```python
from whycon.srv import SetNumberOfTargets
```
* Add the following fuction in code
```python
def service_check(target,thresh_value):
  rospy.wait_for_service('whycon/reset')
  try:
    target_value = rospy.ServiceProxy('whycon/reset', SetNumberOfTargets)
    resp1 = target_value(target,thresh_value)
    return resp1.targets
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
 ```     
> This function takes two arguments: target value and threshold value and return current targets value. 



