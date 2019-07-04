//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    //using System.Globalization;
    //using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Media3D;
    using Microsoft.Kinect;
    using System.IO.Ports;
    using System.Threading;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Active port
        /// </summary>
        private SerialPort port;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // right arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            //// Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            //// Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            //// Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            //set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {            
            this.port = new SerialPort("COM5", 9600, Parity.None, 8, StopBits.One);
            port.Open();

            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
                this.port.Close();
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);
                            Thread tid1 = new Thread(readPort);
                            tid1.Start();

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                /*<<<<<<<appended code>>>>>>>*/
                                double dotProduct;

                                /*Joint position vectors*/
                                Vector3D kneeLeft = new Vector3D(joints[JointType.KneeLeft].Position.X, joints[JointType.KneeLeft].Position.Y, joints[JointType.KneeLeft].Position.Z);
                                Vector3D ankleLeft = new Vector3D(joints[JointType.AnkleLeft].Position.X, joints[JointType.AnkleLeft].Position.Y, joints[JointType.AnkleLeft].Position.Z);
                                Vector3D footLeft = new Vector3D(joints[JointType.FootLeft].Position.X, joints[JointType.FootLeft].Position.Y, joints[JointType.FootLeft].Position.Z);
                                Vector3D kneeRight = new Vector3D(joints[JointType.KneeRight].Position.X, joints[JointType.KneeRight].Position.Y, joints[JointType.KneeRight].Position.Z);
                                Vector3D ankleRight = new Vector3D(joints[JointType.AnkleRight].Position.X, joints[JointType.AnkleRight].Position.Y, joints[JointType.AnkleRight].Position.Z);
                                Vector3D footRight = new Vector3D(joints[JointType.FootRight].Position.X, joints[JointType.FootRight].Position.Y, joints[JointType.FootRight].Position.Z);

                                Vector3D hipRight = new Vector3D(joints[JointType.HipRight].Position.X, joints[JointType.HipRight].Position.Y, joints[JointType.HipRight].Position.Z);
                                Vector3D hipLeft = new Vector3D(joints[JointType.HipLeft].Position.X, joints[JointType.HipLeft].Position.Y, joints[JointType.HipLeft].Position.Z);
                                Vector3D spineMid = new Vector3D(joints[JointType.SpineMid].Position.X, joints[JointType.SpineMid].Position.Y, joints[JointType.SpineMid].Position.Z);
                                Vector3D spineBase = new Vector3D(joints[JointType.SpineBase].Position.X, joints[JointType.SpineBase].Position.Y, joints[JointType.SpineBase].Position.Z);


                                /*Direction Vectors*/
                                Vector3D leftThighVector = kneeLeft - hipLeft;
                                Vector3D leftCalfVector = ankleLeft - kneeLeft;
                                Vector3D leftFootVector = footLeft - ankleLeft;
                                Vector3D rightThighVector = kneeRight - hipRight;
                                Vector3D rightCalfVector = ankleRight - kneeRight;
                                Vector3D rightFootVector = footRight - ankleRight;

                                Vector3D spineVector = spineMid - spineBase;
                                Vector3D hipVector = hipLeft - hipRight;

                                /*<<<Right Leg>>>*/
                                /*Angle of right knee*/
                                rightThighVector.Normalize();
                                rightCalfVector.Normalize();
                                dotProduct = Vector3D.DotProduct(rightThighVector, rightCalfVector);
                                double rightKnee = (double)Math.Acos(dotProduct) / Math.PI * 180;
                                /*Angle of right knee*/

                                /*Angle of right thigh relative to front*/
                                //Hip vector along horizontal plane
                                Vector3D hipRightXZ = new Vector3D(joints[JointType.HipRight].Position.X, 0, joints[JointType.HipRight].Position.Z);
                                Vector3D hipLeftXZ = new Vector3D(joints[JointType.HipLeft].Position.X, 0, joints[JointType.HipLeft].Position.Z);
                                Vector3D hipVectorXZ = hipLeftXZ - hipRightXZ; //hip vector along horizontal plane                              
                                Vector3D hipNormal = Vector3D.CrossProduct(hipVectorXZ, spineVector); //normal to hip vector and spine vector
                                //Ratio to multiply rightThighVectorXZ by such that it forms the right vector 
                                double golden_ratio = hipNormal.Y / Math.Sqrt(Math.Pow(hipNormal.X, 2) + Math.Pow(hipNormal.Z, 2));
                                //Thigh vector along horizontal plane
                                Vector3D kneeRightXZ = new Vector3D(joints[JointType.KneeRight].Position.X, 0, joints[JointType.KneeRight].Position.Z);
                                Vector3D rightThighVectorXZ = kneeRightXZ - hipRightXZ;
                                double rightFrontNormalY = golden_ratio * rightThighVectorXZ.Length;
                                //Reverse rightThighVector if leg is moved backwards
                                double rightThighVectorXZX;
                                double rightThighVectorXZZ;
                                if (rightThighVectorXZ.Z < 0)
                                {
                                    rightThighVectorXZX = rightThighVectorXZ.X;
                                    rightThighVectorXZZ = rightThighVectorXZ.Z;
                                }
                                else
                                {
                                    rightThighVectorXZX = -rightThighVectorXZ.X;
                                    rightThighVectorXZZ = -rightThighVectorXZ.Z;
                                }
                                Vector3D rightFrontNormal = new Vector3D(rightThighVectorXZX, rightFrontNormalY, rightThighVectorXZZ); //problem with X and Z axes is that they change orientation with thighVector
                                //thigh angle relative to front without considering if it's above horizontal plane
                                rightFrontNormal.Normalize();
                                rightThighVector.Normalize();
                                dotProduct = Vector3D.DotProduct(rightThighVector, rightFrontNormal);
                                double rightThighAngleZAbsolute = 90 - Math.Acos(dotProduct) / Math.PI * 180;
                                //Check if thigh is raised higher than plane of horizontal relative to torso
                                spineVector.Normalize();
                                double rightAboveBelow = Vector3D.DotProduct(rightThighVector, spineVector);
                                double rightThighZ;
                                if (rightAboveBelow < 0)
                                {
                                    rightThighZ = rightThighAngleZAbsolute;
                                }
                                else
                                {
                                    rightThighZ = 180 - rightThighAngleZAbsolute;
                                }
                                /*Angle of right thigh relative to front*/

                                /*Angle of right thigh relative to side*/
                                rightThighVector.Normalize();
                                hipVector.Normalize();
                                dotProduct = Vector3D.DotProduct(rightThighVector, -hipVector);
                                double rightThighX = 90 - Math.Acos(dotProduct) / Math.PI * 180;
                                /*Angle of right thigh relative to side*/

                                /*Right leg twist angle*/
                                Vector3D rightThighNormal = Vector3D.CrossProduct(rightThighVector, rightFrontNormal);
                                rightThighNormal.Normalize();
                                rightCalfVector.Normalize();
                                dotProduct = Vector3D.DotProduct(rightThighNormal, rightCalfVector);
                                double rightLegTwist = 90 - Math.Acos(dotProduct) / Math.PI * 180;
                                /*Right leg twist angle*/
                                /*<<<Right Leg>>>*/

                                /*Right foot angle*/
                                Vector3D rightFoot = Vector3D.CrossProduct(rightCalfVector, rightFootVector);
                                rightCalfVector.Normalize();
                                rightFootVector.Normalize();
                                dotProduct = Vector3D.DotProduct(rightCalfVector, rightFootVector);
                                double rightFootAngle = Math.Acos(dotProduct) / Math.PI * 180 - 90;

                                /*<<<Left Leg>>>*/
                                /*Angle of left knee*/
                                leftThighVector.Normalize();
                                leftCalfVector.Normalize();
                                dotProduct = Vector3D.DotProduct(leftThighVector, leftCalfVector);
                                double leftKnee = (double)Math.Acos(dotProduct) / Math.PI * 180;
                                /*Angle of left knee*/

                                /*Angle of left thigh relative to front*/
                                //Thigh vector along horizontal plane
                                Vector3D kneeLeftXZ = new Vector3D(joints[JointType.KneeLeft].Position.X, 0, joints[JointType.KneeLeft].Position.Z);
                                Vector3D leftThighVectorXZ = kneeLeftXZ - hipLeftXZ;
                                double leftFrontNormalY = golden_ratio * leftThighVectorXZ.Length;
                                //Reverse leftThighVector if leg is moved backwards
                                double leftThighVectorXZX;
                                double leftThighVectorXZZ;
                                if (leftThighVectorXZ.Z < 0)
                                {
                                    leftThighVectorXZX = leftThighVectorXZ.X;
                                    leftThighVectorXZZ = leftThighVectorXZ.Z;
                                }
                                else
                                {
                                    leftThighVectorXZX = -leftThighVectorXZ.X;
                                    leftThighVectorXZZ = -leftThighVectorXZ.Z;
                                }
                                Vector3D leftFrontNormal = new Vector3D(leftThighVectorXZX, leftFrontNormalY, leftThighVectorXZZ); //problem with X and Z axes is that they change orientation with thighVector
                                //thigh angle relative to front without considering if it's above horizontal plane
                                leftFrontNormal.Normalize();
                                leftThighVector.Normalize();
                                dotProduct = Vector3D.DotProduct(leftThighVector, leftFrontNormal);
                                double leftThighAngleZAbsolute = 90 - Math.Acos(dotProduct) / Math.PI * 180;
                                //Check if thigh is raised higher than plane of horizontal relative to torso
                                spineVector.Normalize();
                                double LeftAboveBelow = Vector3D.DotProduct(leftThighVector, spineVector);
                                double leftThighZ;
                                if (LeftAboveBelow < 0)
                                {
                                    leftThighZ = leftThighAngleZAbsolute;
                                }
                                else
                                {
                                    leftThighZ = 180 - leftThighAngleZAbsolute;
                                }
                                /*Angle of left thigh relative to front*/

                                /*Angle of left thigh relative to side*/
                                leftThighVector.Normalize();
                                hipVector.Normalize();
                                dotProduct = Vector3D.DotProduct(leftThighVector, hipVector);
                                double leftThighX = 90 - Math.Acos(dotProduct) / Math.PI * 180;
                                /*Angle of left thigh relative to side*/

                                /*Left leg twist angle*/
                                Vector3D leftThighNormal = Vector3D.CrossProduct(leftFrontNormal, leftThighVector);
                                leftThighNormal.Normalize();
                                leftCalfVector.Normalize();
                                dotProduct = Vector3D.DotProduct(leftThighNormal, leftCalfVector);
                                double leftLegTwist = 90 - Math.Acos(dotProduct) / Math.PI * 180;
                                /*Left leg twist angle*/

                                /*Left foot angle*/
                                Vector3D leftFoot = Vector3D.CrossProduct(leftCalfVector, leftFootVector);
                                leftCalfVector.Normalize();
                                leftFootVector.Normalize();
                                dotProduct = Vector3D.DotProduct(leftCalfVector, leftFootVector);
                                double leftFootAngle = Math.Acos(dotProduct) / Math.PI * 180 - 90;
                                /*<<<Left Leg>>>*/

                                Debug.Write(leftThighX + " " + leftThighZ + " " + leftLegTwist + " " + leftKnee + " " + rightThighX + " " + rightThighZ + " " + rightLegTwist + " " + rightKnee + " " + "\n");
                                /*<<<<<<<end appended code>>>>>>>*/

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }

            
        }

        public void readPort()
        {
            int x = 0;
            while (x < 10000)
            {
                string footAngles = port.ReadLine();
                Debug.Write(footAngles + "\n");
                x++;
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                ////appended code;

                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
