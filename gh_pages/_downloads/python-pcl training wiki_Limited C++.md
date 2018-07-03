#Building a Python Service Node

In this exercise, we will fill in the appropriate pieces of code to build a perception pipeline. The end goal will be to broadcast a transform with the pose information of the object of interest.


#Prepare New Workspace:

We will create a new catkin workspace, since this exercise does not overlap with the previous PlanNScan exercises.

1. Disable automatic sourcing of your previous catkin workspace:
    1. <code>gedit ~/.bashrc</code>
    2. comment out <code>#</code> the last line, sourcing your <code>~/catkin_ws/devel/setup.bash</code>

source <code>/opt/ros/kinetic/setup.bash</code>

#Intro (Review Existing Code)

Most of the infrastructure for a ros node has already been completed for you; the focus of this exercise is the perception algorithms/pipleline. The <code>CMakelists.txt</code> and <code>package.xml</code> are complete and an executable has been provided. You could run the executable as is, but you would get errors. At this time we will explore the source code that has been provided - browse the provided <code>perception_node.cpp</code> file. This tutorial is a follow up to training <a href="Exercise 5.1 Building a Perception Pipeline" target="http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline.html">Exercise 5.1 Building a Perception Pipeline</a> and as such the C++ code has already been set up.  Open up the preception_node.cpp file and look over the filtering functions.

#Create a Python node

Now that we have converted several filters to C++ functions, we are ready to call it from a Python node.  If you have not done so already, install PyCharm, community edition. <u>Maybe say if desired? Because it is not necessary</>

1.	Create a new package inside your perception_ws. In the terminal, change directory to your src folder:
    <pre><code>
$ cd ~/perception_ws/src/
$ catkin_create_pkg filter_call rospy roscpp perception_msgs</code></pre> 

    <u>Using catkin_make instead of catkin_tools? i.e. "catkin_create_pkg" instead of "catkin create pkg". I think the rest of tutorials use catkin_tools. Still works, regardless.</u>
    
    <u>\*Returning back to this point after going further in the tutorial, I have issues finding perception_msgs. It throws an error, but it can find sensor_msgs, which seems to be the message package we are depending on anyway. Currently working having swapped the two<u>

2.	Check that your package was created:
    <pre><code>
$ ls</code></pre>

    You can open the file in Pycharm or QT (or you can use nano, emacs, vim, or sublime)

3.	Edit line 2 of CMakesList.txt to define the name of the project:
<pre><code>
$ project(filter_call)</code></pre>

4.	Edit line 7-10:
<pre><code>
find_package(catkin REQUIRED COMPONENTS
perception_msgs
roscpp
rospy
)</code></pre>

    <u>The last two steps should be unecessary if the user typed in the catkin_create_pkg command correctly as those lines will already be filled correctly into the CMakeLists<u>


    Specify the packages your package depends on.  *We will not be using ‘perception_msgs’ as we will not be creating custom messages in this course.* <-- <u>Self contrdicting sentence. I'm not sure what is correct.</u> It is included for further student knowledge. If you wish for a more in depth explanation including how to implement custom messages, here is a good <a href="MIT resource" target="http://duckietown.mit.edu/media/pdfs/1rpRisFoCYUm0XT78j-nAYidlh-cDtLCdEbIaBCnx9ew.pdf">MIT resource</a> on the steps taken

5.	Uncomment line 23 and save.
<pre><code>
catkin_python_setup()</code></pre>

6.	Next, we need to update the package.xml file to allow Python communication to ROS.  Open your package.xml file in QT and begin on line 3 by updating the package name
<pre><code>
<description>The lesson_perception package</description></code></pre>

    <u>I don't think this is correct. This is not the lesson_preception package. I believe this should be left unchanged</u>

7.	Update your <build_depend> and <run_depend> to reflect roscpp, rospy, and perception_msgs.

    <u>Again, this should already have been done in the initial creation of the package</u>

8.	Save and close the file.

    <u>Overall, it seems that package.xml should not need any initial edits<u>

#<big>Creating setup.py</big>

The <code>setup.py</code> file makes your python module available to the entire workspace and subsequent packaged.  By default, this isn’t created by the <code>catkin_create_pkg</code> script.

1.	In your terminal type
<pre><code>
$ nano filter_call/setup.py</code></pre>

    <u>Maybe say create the file and edit it instead of specifying nano?</u>

2.	Copy and paste the following to the setup.py file (to paste into a terminal, Ctrl+Shift+V)

```python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
packages=[''],
package_dir={'': 'include'},
)
setup(**setup_args)
```


Change <code>packages=[''],</code> to your list of strings of the name of the folders inside your *include* folder.  By convention, this will be the same name as the package <code>'filter_call'</code> . *The configures ‘filter_call’/include/filter_call as a python module available to the whole workspace.* <-- <u>What do you mean by this?</u>

3.	Save and close the file.

    In order for this folder to be treated as a python module, the <code>\__init__.py</code> file must exist.

4.	Create one in the terminal by typing:
    <pre><code>
$ touch filter_call/include/filter_call/\__init__.py</code></pre>

5.	Now we are ready to start developing the client node to call our C++ service.  Create a service called ‘FilterCloud.srv’ as outlined in Section 2.0, updating the <code>CMakeLists.txt</code> and the <code>package.xml</code> file respectively.  Copy and paste the following into the file.

```
#request
sensor_msgs/PointCloud2 input_cloud
string topic
string pcdfilename

# Removes objects outside a defined grid pattern x,y,z
byte VOXELGRID=0

# Removes the objects based on volume of space
byte PASSTHROUGH=1

# Issolate objects located along the largest flat surface (floor)
byte PLANESEGMENTATION=2

# Determine clusters based on pcd density to identify multiple objects
byte CLUSTEREXTRACTION=3

# Operation to be performed
byte operation

---
#response
sensor_msgs/PointCloud2 output_cloud
bool success
```


*Be sure to include the header file associated with this service.* <-- <u>Where? perception_node.cpp?</u>

#Publishing the Point Cloud


As iterated before, we are creating a ROS C++ node to publish the point cloud and then have a Python node listen and call filtering operations, resulting in a new, aggregated point cloud.  Let’s start with converting our C++ code to publish in a manner supportive to python. Remember, the C++ code is already done so all you need to do is write your python script and view the results in rviz.

#<big>Implement a Voxel Filter</big>

In perception_node.cpp in the lesson_perception package, create a function called <code>filterCallBack()</code> that performs the service. This will be used by the python client to run filtering operations.

1.	Above your <code>main</code>, after your filter includes, uncomment the code below.

```c++
bool filterCallback(lesson_perception::FilterCloud::Request& request,
                    lesson_perception::FilterCloud::Response& response)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (request.pcdfilename.empty())
  {
    pcl::fromROSMsg(request.input_cloud, *cloud);
    ROS_INFO_STREAM("cloud size: " << cloud->size());
    if (cloud->empty())
    {
      ROS_ERROR("input cloud empty");
      response.success = false;
      return false;
    }
    //response.output_cloud.header=request.input_cloud.header;
  }
  else
  {
    pcl::io::loadPCDFile(request.pcdfilename, *cloud);
    //response.output_cloud.header.frame_id="kinect_link";
  }

}
```


Now that we have the framework for the filtering, save the file and open your *filter_call.py* <-- <u>From where?</u> and find:

<pre><code>
        # =======================
        # FILL CODE: VOXEL FILTER
        # =======================</code></pre>


Copy and paste the following inside the <code>try:</code>function:

<pre><code>
        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = rospy.get_param('~pcdfilename', '')
        req.operation = 0
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = PointCloud2()

        # ERROR HANDLING
        if '.pcdfilename' == '':
            print('no file found')
            raise Exception('no file found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_voxel = srvp(req)
        print('response received')
        if res_voxel.success == False:
            res_voxel.success = True
            raise Exception('execution not valid')

        # PUBLISH VOXEL FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_voxelGrid', PointCloud2, queue_size=10, latch=True)
        pub.publish(res_voxel.output_cloud)
        print("published: voxel grid filter response")
        rate = rospy.Rate(10)</code></pre>


Uncomment and save.

<pre><code>
        rospy.init_node('filter_cloud', anonymous=True)
        rospy.wait_for_service('filter_cloud')</code></pre>


#<big>Viewing Results</big>

1. run
<pre><code>
roscore</code></pre>

2. Source a new terminal
<pre><code>
rosrun lesson_perception perception_node</code></pre>

3. Source a new terminal
<pre><code>
rosrun filter_call listener _pcdfilename:="/me/ros-industrial/catkin_ws/table.pcd"</code></pre>

4. Source a new terminal
<pre><code>
rosrun rviz rviz</code></pre>

5. Add a new PointCloud2

6. Change the fixed frame to kinect_link

    You may need to uncheck and recheck the PointCloud2.


#Implement Pass-Through Filters

2. In the <code>main():</code> uncomment

<pre><code>
  priv_nh_.param<double>("passThrough_max", passThrough_max_, 1.0f);
  priv_nh_.param<double>("passThrough_min", passThrough_min_, -1.0f);</code></pre>


3. Update the switch to look as shown below:

<pre><code>
  switch (request.operation)
  {

    case lesson_perception::FilterCloud::Request::VOXELGRID :
    {
      filtered_cloud = voxelGrid(cloud, 0.01);
      break;
    }

    case lesson_perception::FilterCloud::Request::PASSTHROUGH :
    {
      filtered_cloud = passThrough(cloud);
      break;
    }
    default :
    {
      ROS_ERROR("no point cloud found");
      return false;
    }

   }</code></pre>


However, because we can pass the file as two types, we need to check that there is even data so above the switch inside the <code>filterCallback()</code> function, add the following code:

<pre><code>
  if (request.pcdfilename.empty())
  {
    pcl::fromROSMsg(request.input_cloud, *cloud);
    ROS_INFO_STREAM("cloud size: " <<cloud->size());
    if (cloud->empty())
    {
      ROS_ERROR("input cloud empty");
      response.success = false;
      return false;
    }
  }
  else
  {
    pcl::io::loadPCDFile(request.pcdfilename, *cloud);
  }
  case lesson_perception::FilterCloud::Request::VOXELGRID :
  {
    filtered_cloud = voxelGrid(cloud, 0.01);
    break;
  }</code></pre>


This checks for data based on both service types.

4. Add variable declarations at the top of the program and uncomment the pcl header similar to the voxel filter.
5. Save and build


#<big>Edit the Python Code</big>

Open the python node and find

<pre><code>
        # =======================
        # FILL CODE: PASSTHROUGH FILTER
        # =======================</code></pre>


6. Copy paste the following code.  Keep care to maintain indents:

<pre><code>
        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = 1
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_voxel.output_cloud

        # ERROR HANDLING
        if '.pcdfilename' == '':
            print('no file found')
            raise Exception('no file found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_pass = srvp(req)
        print('response received')
        if res_pass.success == False:
            res_pass.success = True
            raise Exception('execution not valid')

        # PUBLISH PASSTHROUGH FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_passThrough', PointCloud2, queue_size=10, latch=True)
        pub.publish(res_pass.output_cloud)
        print("published: pass through filter response")
        rate = rospy.Rate(10)</code></pre>


7. Save and run from the terminal

Within Rviz, compare PointCloud2 displays based on the /kinect/depth_registered/points (original camera data) and object_cluster (latest processing step) topics. Part of the original point cloud has been “clipped” out of the latest processing result.

When you are satisfied with the pass-through filter results, press Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.


#Plane Segmentation

This method is one of the most useful for any application where the object is on a flat surface. In order to isolate the objects on a table, you perform a plane fit to the points, which finds the points which comprise the table, and then subtract those points so that you are left with only points corresponding to the object(s) above the table. This is the most complicated PCL method we will be using and it is actually a combination of two: the RANSAC segmentation model, and the extract indices tool. An in depth example can be found on the <a href="PCL Plane Model Segmentation Tutorial" target="http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation">PCL Plane Model Segmentation Tutorial</a>; otherwise you can copy the below code snippet.


1. In the <code>main():</code> uncomment
<pre><code>

  priv_nh_.param<double>("maxIterations", maxIterations_, 200.0f);
  priv_nh_.param<double>("distThreshold", distThreshold_, 0.01f);</code></pre>


2. Update the switch to look as shown below:

<pre><code>
  switch (request.operation)
  {

    case lesson_perception::FilterCloud::Request::VOXELGRID :
    {
      filtered_cloud = voxelGrid(cloud, 0.01);
      break;
    }

    case lesson_perception::FilterCloud::Request::PASSTHROUGH :
    {
      filtered_cloud = passThrough(cloud);
      break;
    }
    case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
    {
      filtered_cloud = planeSegmentation(cloud);
      break;
    }
    default :
    {
      ROS_ERROR("no point cloud found");
      return false;
    }

   }</code></pre>


3. Add variable declarations at the top up the program and uncomment the pcl header.
4. Save and build

#<big>Edit the Python Code</big>

Open the python node and find

<pre><code>
        # =======================
        # FILL CODE: PLANE SEGMENTATION
        # =======================</code></pre>


5. Copy paste the following code.  Keep care to maintain indents:

<pre><code>
        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = 2
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_pass.output_cloud

        # ERROR HANDLING
        if '.pcdfilename' == '':
            print('no file found')
            raise Exception('no file found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_seg = srvp(req)
        print('response received')
        if res_seg.success == False:
            res_seg.success = True
            raise Exception('execution not valid')

        # PUBLISH PLANESEGMENTATION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_planeSegmentation', PointCloud2, queue_size=10, latch=True)
        pub.publish(res_seg.output_cloud)
        print("published: plane segmentation filter response")
        rate = rospy.Rate(10)</code></pre>


6. Save and run from the terminal

Within Rviz, compare PointCloud2 displays based on the /kinect/depth_registered/points (original camera data) and object_cluster (latest processing step) topics. Only points lying above the table plane remain in the latest processing result.

    1. When you are done viewing the results you can go back and change the ”setMaxIterations” and “setDistanceThreshold” values to control how tightly the plane-fit classifies data as inliers/outliers, and view the results again. Try using values of <code>MaxIterations=100</code> and <code>DistanceThreshold=0.010</code>

    2. When you are satisfied with the plane segmentation results, use Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.



#Euclidian Cluster Extraction

This method is useful for any application where there are multiple objects. This is also a complicated PCL method. An in depth example can be found on the <a href="PCL Euclidean Cluster Extration Tutorial" target="http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction">PCL Euclidean Cluster Extration Tutorial</a>.


1. In the <code>main():</code> uncomment

<pre><code>
  priv_nh_.param<double>("clustTol", clustTol_, 0.01f);
  priv_nh_.param<double>("clustMax", clustMax_, 10000.0);
  priv_nh_.param<double>("clustMin", clustMin_, 300.0f);</code></pre>


2. Update the switch to look as shown below:

<pre><code>
  switch (request.operation)
  {

    case lesson_perception::FilterCloud::Request::VOXELGRID :
    {
      filtered_cloud = voxelGrid(cloud, 0.01);
      break;
    }

    case lesson_perception::FilterCloud::Request::PASSTHROUGH :
    {
      filtered_cloud = passThrough(cloud);
      break;
    }
    case lesson_perception::FilterCloud::Request::PLANESEGMENTATION :
    {
      filtered_cloud = planeSegmentation(cloud);
      break;
    }
    case lesson_perception::FilterCloud::Request::CLUSTEREXTRACTION :
    {
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> temp =clusterExtraction(cloud);
      if (temp.size()>0)
      {
        filtered_cloud = temp[0];
      }
      //filtered_cloud = clusterExtraction(cloud)[0];
      break;
    }
    default :
    {
      ROS_ERROR("no point cloud found");
      return false;
    }

   }</code></pre>


3. Add variable declarations at the top up the program and uncomment the pcl header.
4. Save and build


#<big>Edit the Python Code</big>

Open the python node and find

<pre><code>
        # =======================
        # FILL CODE: CLUSTER EXTRACTION
        # =======================</code></pre>


5. Copy paste the following code.  Keep care to maintain indents:

<pre><code>
        srvp = rospy.ServiceProxy('filter_cloud', lesson_perception.srv.FilterCloud)
        req = lesson_perception.srv.FilterCloudRequest()
        req.pcdfilename = ''
        req.operation = 3
        # FROM THE SERVICE, ASSIGN POINTS
        req.input_cloud = res_seg.output_cloud

        # ERROR HANDLING
        if '.pcdfilename' == '':
            print('no file found')
            raise Exception('no file found')

        # PACKAGE THE FILTERED POINTCLOUD2 TO BE PUBLISHED
        res_cluster = srvp(req)
        print('response received')
        if res_cluster.success == False:
            res_cluster.success = True
            raise Exception('execution not valid')

        # PUBLISH CLUSTEREXTRACTION FILTERED POINTCLOUD2
        pub = rospy.Publisher('/perception_clusterExtraction', PointCloud2, queue_size=10, latch=True)
        pub.publish(res_cluster.output_cloud)
        print("published: cluster extraction filter response")
        rate = rospy.Rate(10)</code></pre>


6. Save and run from the terminal

    1. When you are satisfied with the cluster extraction results, use Ctrl+C to kill the node. There is no need to close or kill the other terminals/nodes.


#Future Study

The student is encouraged to convert <a href="Exercise 5.1" target="http://ros-industrial.github.io/industrial_training/_source/session5/Building-a-Perception-Pipeline.html">Exercise 5.1</a> into callable functions and further refine the filtering operations.

Futher, for simplicity the python code was repeated for each filtering instance, the student is encouraged to create a function to handle the publishing.




































