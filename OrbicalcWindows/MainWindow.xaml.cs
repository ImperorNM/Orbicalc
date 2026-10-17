using OrbicalcLib;
using OrbicalcLib.Classes;
using OrbicalcLib.Classes.Navigation;
using OrbicalcLib.Data;
using OrbicalcLib.Handlers;
using OrbicalcLib.Interfaces;
using OrbicalcLib.Modules;
using OrbicalcLib.Util;
using PeterO.Numbers;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace OrbicalcWindows
{
    public partial class MainWindow : Window
    {
        private readonly List<UIElement> _maneuverNodeUIElements = [];

        private WriteableBitmap? _trailBitmap;
        private byte[]? _pixelBuffer;

        private readonly PhysicalEngine _physicalEngine;
        private readonly IObjectHandler _simObjectHandler;
        private readonly ISingleObjectHandler _spaceshipHandler;
        private readonly Stopwatch _fpsStopwatch = new();

        private int _frameCount = 0;
        private double _lastFpsTime = 0;

        private long _lastEngineTicks = 0;
        private double _lastUpsTime = 0;

        private double _zoomLevel = 1.0;
        private Point _panOffset = new Point(0, 0);
        private bool _isDragging = false;
        private Point _lastMousePosition;
        private string _focusTarget = "Sun";

        private readonly List<Ellipse> _planetShapes = new();
        private readonly List<TextBlock> _planetLabels = new();
        private readonly List<Line> _velLines = new();
        private readonly List<Line> _accLines = new();
        private Polygon? _shipPolygon;
        private Polyline? _trajectoryPolyline;

        private readonly List<ManeuverNode> _activeManeuverNodes = new();
        private List<BigFloatVector2> _cachedPredictedPoints = new();
        private long _lastTrajectoryUpdateTick = -1;
        private bool _isPredictingTrajectory = false;

        public MainWindow()
        {
            InitializeComponent();

            _simObjectHandler = new SimObjectHandler();
            _spaceshipHandler = new SpaceshipHandler();

            AddSolarSystem(_simObjectHandler);
            SpawnSpaceshipNearEarth();

            var gravityModule = new GravityModule(_simObjectHandler);
            var spaceshipGravityModule = new SpaceshipGravityModule(_simObjectHandler, _spaceshipHandler);

            _physicalEngine = new PhysicalEngine(gravityModule, spaceshipGravityModule);

            InitializeVisualElements();

            ToggleSimulationButton.Content = "Старт";
            ToggleSimulationButton.Background = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#28A745"));

            Closing += MainWindow_Closing;

            _fpsStopwatch.Start();
            CompositionTarget.Rendering += OnRenderFrame;

            GravityAssistPlanner.DiagnosticLambertTest(_simObjectHandler.Objects, 10000);
        }

        private void InitializeVisualElements()
        {
            MainCanvas.Children.Clear();
            _planetShapes.Clear();
            _planetLabels.Clear();
            _velLines.Clear();
            _accLines.Clear();

            var objects = _simObjectHandler.Objects;
            string[] names = { "Sun", "Mercury", "Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune" };

            // 1. Линия для отрисовки предсказанной орбиты
            _trajectoryPolyline = new Polyline
            {
                Stroke = Brushes.Cyan,
                StrokeThickness = 1.2,
                StrokeDashArray = new DoubleCollection { 3, 3 },
                Opacity = 0.85
            };
            MainCanvas.Children.Add(_trajectoryPolyline);

            // 2. Элементы для каждой планеты
            for (int i = 0; i < objects.Length; i++)
            {
                var velLine = new Line { Stroke = Brushes.LimeGreen, StrokeThickness = 1.0, Visibility = Visibility.Collapsed };
                var accLine = new Line { Stroke = Brushes.Red, StrokeThickness = 1.2, Visibility = Visibility.Collapsed };

                var ellipse = new Ellipse
                {
                    Stroke = Brushes.White,
                    StrokeThickness = 0.8
                };

                var label = new TextBlock
                {
                    Text = i < names.Length ? names[i] : $"Obj {i}",
                    Foreground = Brushes.LightGray,
                    FontSize = 10,
                    IsHitTestVisible = false
                };

                _velLines.Add(velLine);
                _accLines.Add(accLine);
                _planetShapes.Add(ellipse);
                _planetLabels.Add(label);

                MainCanvas.Children.Add(velLine);
                MainCanvas.Children.Add(accLine);
                MainCanvas.Children.Add(ellipse);
                MainCanvas.Children.Add(label);
            }

            // 3. Форма для космического корабля
            _shipPolygon = new Polygon
            {
                Fill = Brushes.Magenta,
                Stroke = Brushes.White,
                StrokeThickness = 1.0,
                Points = new PointCollection
                {
                    new Point(0, -7),
                    new Point(-4.5, 7),
                    new Point(4.5, 7)
                }
            };
            MainCanvas.Children.Add(_shipPolygon);
        }

        private void CanvasContainer_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            int width = (int)CanvasContainer.ActualWidth;
            int height = (int)CanvasContainer.ActualHeight;

            if (width <= 0 || height <= 0) return;

            _trailBitmap = new WriteableBitmap(width, height, 96, 96, PixelFormats.Bgra32, null);
            _pixelBuffer = new byte[width * height * 4];
            TrailImage.Source = _trailBitmap;
        }

        public void AddSolarSystem(IObjectHandler objectHandler)
        {
            var bodies = new (string Name, string DistanceMeters, string VelocityMs, string MassKg, string RadiusMeters)[]
            {
                ("Sun",     "0",             "0",     "1.989e30", "696340000"),
                ("Mercury", "57910000000",   "47360", "3.301e23", "2439700"),
                ("Venus",   "108200000000",  "35020", "4.867e24", "6051800"),
                ("Earth",   "149600000000",  "29780", "5.972e24", "6371000"),
                ("Mars",    "227940000000",  "24070", "6.417e23", "3389500"),
                ("Jupiter", "778330000000",  "13070", "1.898e27", "69911000"),
                ("Saturn",  "1429400000000", "9680",  "5.683e26", "58232000"),
                ("Uranus",  "2870990000000", "6800",  "8.681e25", "25362000"),
                ("Neptune", "4504300000000", "5430",  "1.024e26", "24622000")
            };

            foreach (var b in bodies)
            {
                var body = new SimObject(
                    new BigFloatVector2(EDecimal.FromString(b.DistanceMeters), EDecimal.Zero),
                    new BigFloatVector2(EDecimal.Zero, EDecimal.FromString(b.VelocityMs)),
                    EDecimal.FromString(b.MassKg),
                    EDecimal.FromString(b.RadiusMeters)
                );
                objectHandler.Add(body);
            }
        }

        private void SpawnSpaceshipNearEarth()
        {
            var objects = _simObjectHandler.Objects;
            if (objects.Length < 4) return;

            var earth = objects[3];
            var offset = EDecimal.FromString("5000000000"); // 5 млн км
            var spaceshipPos = new BigFloatVector2(earth.Position.X + offset, earth.Position.Y);

            var spaceship = new SimObject(
                spaceshipPos,
                earth.Velocity,
                EDecimal.FromString("1000"),
                EDecimal.FromString("10")
            );

            _spaceshipHandler.Object = spaceship;
        }

        private Point MapToScreen(double posX, double posY, double focusX, double focusY, double canvasCenterX, double canvasCenterY, double baseScale)
        {
            double relX = posX - focusX;
            double relY = posY - focusY;
            double realRadius = Math.Sqrt(relX * relX + relY * relY);

            if (realRadius < 1.0)
            {
                return new Point(canvasCenterX, canvasCenterY);
            }

            double angle = Math.Atan2(relY, relX);
            double displayRadiusScaled = Math.Pow(realRadius, 0.35) * baseScale;

            double screenX = canvasCenterX + (displayRadiusScaled * Math.Cos(angle));
            double screenY = canvasCenterY - (displayRadiusScaled * Math.Sin(angle));

            return new Point(screenX, screenY);
        }

        private void DrawManeuverNodes(double focusX, double focusY, double canvasCenterX, double canvasCenterY, double baseScale)
        {
            foreach (var element in _maneuverNodeUIElements)
            {
                MainCanvas.Children.Remove(element);
            }
            _maneuverNodeUIElements.Clear();

            if (_activeManeuverNodes.Count == 0 || _cachedPredictedPoints == null || _cachedPredictedPoints.Count == 0)
                return;

            long currentTick = _physicalEngine.TotalTicks;

            for (int i = 0; i < _activeManeuverNodes.Count; i++)
            {
                var node = _activeManeuverNodes[i];
                if (node.Executed) continue;

                long ticksFromNow = node.TargetTick - currentTick;

                if (ticksFromNow < 0) continue;

                int stepIndex = (int)ticksFromNow;

                if (stepIndex >= _cachedPredictedPoints.Count)
                {
                    stepIndex = _cachedPredictedPoints.Count - 1;
                }

                var targetPos = _cachedPredictedPoints[stepIndex];
                Point screenPos = MapToScreen(targetPos.X.ToDouble(), targetPos.Y.ToDouble(), focusX, focusY, canvasCenterX, canvasCenterY, baseScale);

                var nodeCircle = new Ellipse
                {
                    Width = 14,
                    Height = 14,
                    Stroke = Brushes.Orange,
                    StrokeThickness = 2,
                    Fill = new SolidColorBrush(Color.FromArgb(120, 255, 165, 0)),
                    IsHitTestVisible = false
                };

                Canvas.SetLeft(nodeCircle, screenPos.X - 7);
                Canvas.SetTop(nodeCircle, screenPos.Y - 7);

                var nodeLabel = new TextBlock
                {
                    Text = $"Node #{i + 1}\nΔv: {node.DeltaV.Magnitude.ToDouble():F1} m/s",
                    Foreground = Brushes.Orange,
                    FontSize = 10,
                    FontWeight = FontWeights.Bold,
                    IsHitTestVisible = false
                };

                double offsetY = (i % 2 == 0) ? -25 : 12;

                Canvas.SetLeft(nodeLabel, screenPos.X + 12);
                Canvas.SetTop(nodeLabel, screenPos.Y + offsetY);

                MainCanvas.Children.Add(nodeCircle);
                MainCanvas.Children.Add(nodeLabel);

                _maneuverNodeUIElements.Add(nodeCircle);
                _maneuverNodeUIElements.Add(nodeLabel);
            }
        }

        private void OnRenderFrame(object? sender, EventArgs e)
        {
            RenderObjects();
            UpdateMetrics();
            UpdateDebugInfo();
        }

        private void RenderObjects()
        {
            double width = MainCanvas.ActualWidth > 0 ? MainCanvas.ActualWidth : CanvasContainer.ActualWidth;
            double height = MainCanvas.ActualHeight > 0 ? MainCanvas.ActualHeight : CanvasContainer.ActualHeight;

            if (width <= 0 || height <= 0) return;

            var objects = _simObjectHandler.Objects;
            if (objects.Length == 0) return;

            // Выбираем центр отсчета для камеры
            double focusX = 0, focusY = 0;
            var sunObj = objects[0];

            switch (_focusTarget)
            {
                case "Earth":
                    if (objects.Length > 3)
                    {
                        focusX = objects[3].Position.X.ToDouble();
                        focusY = objects[3].Position.Y.ToDouble();
                    }
                    break;
                case "Ship":
                    if (_spaceshipHandler.Object != null)
                    {
                        focusX = _spaceshipHandler.Object.Position.X.ToDouble();
                        focusY = _spaceshipHandler.Object.Position.Y.ToDouble();
                    }
                    break;
                case "Free":
                    focusX = sunObj.Position.X.ToDouble();
                    focusY = sunObj.Position.Y.ToDouble();
                    break;
                default: // Sun
                    focusX = sunObj.Position.X.ToDouble();
                    focusY = sunObj.Position.Y.ToDouble();
                    break;
            }

            double canvasCenterX = (width / 2.0) + _panOffset.X;
            double canvasCenterY = (height / 2.0) + _panOffset.Y;

            (double DisplayRadius, Brush Color)[] visualStyle = [
                (16.0, Brushes.Gold),         // Sun
                (5.0,  Brushes.DarkGray),     // Mercury
                (7.0,  Brushes.Khaki),        // Venus
                (7.5,  Brushes.DeepSkyBlue),  // Earth
                (6.0,  Brushes.IndianRed),    // Mars
                (12.0, Brushes.Orange),       // Jupiter
                (10.0, Brushes.Goldenrod),    // Saturn
                (8.5,  Brushes.LightSkyBlue), // Uranus
                (8.5,  Brushes.RoyalBlue)     // Neptune
            ];

            double maxNeptuneDistance = 4.5e11;
            double baseScale = ((Math.Min(width, height) / 3.5) / Math.Pow(maxNeptuneDistance, 0.35)) * _zoomLevel;

            bool showVectors = ShowVectorsCheckBox.IsChecked ?? false;

            for (int i = 0; i < objects.Length; i++)
            {
                var obj = objects[i];

                double posX = obj.Position.X.ToDouble();
                double posY = obj.Position.Y.ToDouble();

                double relX = posX - focusX;
                double relY = posY - focusY;
                double realRadius = Math.Sqrt(relX * relX + relY * relY);

                double screenX, screenY;

                if (realRadius < 1.0)
                {
                    screenX = canvasCenterX;
                    screenY = canvasCenterY;
                }
                else
                {
                    double angle = Math.Atan2(relY, relX);
                    double displayRadiusScaled = Math.Pow(realRadius, 0.35) * baseScale;

                    screenX = canvasCenterX + (displayRadiusScaled * Math.Cos(angle));
                    screenY = canvasCenterY - (displayRadiusScaled * Math.Sin(angle));
                }

                var (displayRadius, fillBrush) = i < visualStyle.Length ? visualStyle[i] : (6.0, Brushes.White);

                // Векторы V и A
                if (showVectors)
                {
                    double velX = obj.Velocity.X.ToDouble();
                    double velY = obj.Velocity.Y.ToDouble();
                    double accX = obj.Acceleration.X.ToDouble();
                    double accY = obj.Acceleration.Y.ToDouble();

                    _velLines[i].X1 = screenX;
                    _velLines[i].Y1 = screenY;
                    _velLines[i].X2 = screenX + (velX * 0.0005 * _zoomLevel);
                    _velLines[i].Y2 = screenY - (velY * 0.0005 * _zoomLevel);
                    _velLines[i].Visibility = Visibility.Visible;

                    _accLines[i].X1 = screenX;
                    _accLines[i].Y1 = screenY;
                    _accLines[i].X2 = screenX + (accX * 10000.0 * _zoomLevel);
                    _accLines[i].Y2 = screenY - (accY * 10000.0 * _zoomLevel);
                    _accLines[i].Visibility = Visibility.Visible;
                }
                else
                {
                    _velLines[i].Visibility = Visibility.Collapsed;
                    _accLines[i].Visibility = Visibility.Collapsed;
                }

                // Планета
                var ellipse = _planetShapes[i];
                ellipse.Width = displayRadius * 2;
                ellipse.Height = displayRadius * 2;
                ellipse.Fill = fillBrush;

                Canvas.SetLeft(ellipse, screenX - displayRadius);
                Canvas.SetTop(ellipse, screenY - displayRadius);

                // Метка с названием
                var label = _planetLabels[i];
                Canvas.SetLeft(label, screenX + displayRadius + 2);
                Canvas.SetTop(label, screenY - displayRadius);
            }

            DrawSpaceship(focusX, focusY, canvasCenterX, canvasCenterY, baseScale);
            DrawPredictedTrajectory(focusX, focusY, canvasCenterX, canvasCenterY, baseScale);
            DrawManeuverNodes(focusX, focusY, canvasCenterX, canvasCenterY, baseScale);
        }

        private void DrawSpaceship(double focusX, double focusY, double canvasCenterX, double canvasCenterY, double baseScale)
        {
            var spaceship = _spaceshipHandler.Object;
            if (spaceship == null || _shipPolygon == null) return;

            double shipX = spaceship.Position.X.ToDouble();
            double shipY = spaceship.Position.Y.ToDouble();

            double relX = shipX - focusX;
            double relY = shipY - focusY;
            double realRadius = Math.Sqrt(relX * relX + relY * relY);

            double screenX, screenY;

            if (realRadius < 1.0)
            {
                screenX = canvasCenterX;
                screenY = canvasCenterY;
            }
            else
            {
                double angle = Math.Atan2(relY, relX);
                double displayRadiusScaled = Math.Pow(realRadius, 0.35) * baseScale;

                screenX = canvasCenterX + (displayRadiusScaled * Math.Cos(angle));
                screenY = canvasCenterY - (displayRadiusScaled * Math.Sin(angle));
            }

            FadeAndDrawTrail(screenX, screenY);

            Canvas.SetLeft(_shipPolygon, screenX);
            Canvas.SetTop(_shipPolygon, screenY);
        }

        private void DrawPredictedTrajectory(double focusX, double focusY, double canvasCenterX, double canvasCenterY, double baseScale)
        {
            if (_trajectoryPolyline == null) return;

            bool showTrajectory = ShowTrajectoryCheckBox.IsChecked ?? false;
            var ship = _spaceshipHandler.Object;

            if (!showTrajectory || ship == null)
            {
                _trajectoryPolyline.Visibility = Visibility.Collapsed;
                return;
            }

            _trajectoryPolyline.Visibility = Visibility.Visible;

            long currentTick = _physicalEngine.TotalTicks;
            if ((currentTick - _lastTrajectoryUpdateTick >= 10 || _lastTrajectoryUpdateTick == -1) && !_isPredictingTrajectory)
            {
                _lastTrajectoryUpdateTick = currentTick;
                _isPredictingTrajectory = true;

                var shipCopy = new SimObject(ship.Position, ship.Velocity, ship.Mass, ship.Size);
                var planetsCopy = new SimObject[_simObjectHandler.Objects.Length];
                for (int i = 0; i < planetsCopy.Length; i++)
                {
                    var p = _simObjectHandler.Objects[i];
                    planetsCopy[i] = new SimObject(p.Position, p.Velocity, p.Mass, p.Size);
                }

                double dtSeconds = _physicalEngine.FixedDeltaTime.TotalSeconds;
                var nodesCopy = new List<ManeuverNode>(_activeManeuverNodes);

                Task.Run(() =>
                {
                    int totalSteps = 300;
                    if (nodesCopy.Count > 0)
                    {
                        long maxTick = 0;
                        foreach (var node in nodesCopy)
                        {
                            if (node.TargetTick > maxTick)
                                maxTick = node.TargetTick;
                        }

                        int requiredSteps = (int)(maxTick - currentTick) + 150;
                        totalSteps = Math.Max(300, requiredSteps);
                    }

                    var points = TrajectoryPredictor.PredictTrajectory(
                        shipCopy,
                        planetsCopy,
                        nodesCopy,
                        currentTick,
                        EDecimal.FromDouble(dtSeconds),
                        totalSteps
                    );

                    Dispatcher.InvokeAsync(() =>
                    {
                        _cachedPredictedPoints = points;
                        _isPredictingTrajectory = false;
                    });
                });
            }

            var pointCollection = new PointCollection(_cachedPredictedPoints.Count);

            foreach (var pos in _cachedPredictedPoints)
            {
                double px = pos.X.ToDouble();
                double py = pos.Y.ToDouble();

                double relX = px - focusX;
                double relY = py - focusY;
                double realRadius = Math.Sqrt(relX * relX + relY * relY);

                if (realRadius < 1.0)
                {
                    pointCollection.Add(new Point(canvasCenterX, canvasCenterY));
                }
                else
                {
                    double angle = Math.Atan2(relY, relX);
                    double displayRadiusScaled = Math.Pow(realRadius, 0.35) * baseScale;

                    double sx = canvasCenterX + (displayRadiusScaled * Math.Cos(angle));
                    double sy = canvasCenterY - (displayRadiusScaled * Math.Sin(angle));
                    pointCollection.Add(new Point(sx, sy));
                }
            }

            _trajectoryPolyline.Points = pointCollection;
        }

        private void FadeAndDrawTrail(double currentScreenX, double currentScreenY)
        {
            if (_trailBitmap == null || _pixelBuffer == null) return;

            int width = _trailBitmap.PixelWidth;
            int height = _trailBitmap.PixelHeight;
            int stride = width * 4;

            Span<byte> bufferSpan = _pixelBuffer.AsSpan();

            for (int i = 3; i < bufferSpan.Length; i += 4)
            {
                if (bufferSpan[i] > 0)
                {
                    bufferSpan[i] = (byte)Math.Max(0, bufferSpan[i] - 2);
                }
            }

            int cx = (int)currentScreenX;
            int cy = (int)currentScreenY;

            if (cx >= 0 && cx < width && cy >= 0 && cy < height)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dy = -1; dy <= 1; dy++)
                    {
                        int px = cx + dx;
                        int py = cy + dy;

                        if (px >= 0 && px < width && py >= 0 && py < height)
                        {
                            int index = (py * width + px) * 4;
                            bufferSpan[index] = 255;     // Blue
                            bufferSpan[index + 1] = 0;   // Green
                            bufferSpan[index + 2] = 255; // Red
                            bufferSpan[index + 3] = 255; // Alpha
                        }
                    }
                }
            }

            _trailBitmap.WritePixels(new Int32Rect(0, 0, width, height), _pixelBuffer, stride, 0);
        }

        private void RenderGrid_MouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Right || e.ChangedButton == MouseButton.Middle)
            {
                _isDragging = true;
                _lastMousePosition = e.GetPosition(RenderGrid);
                RenderGrid.CaptureMouse();
            }
        }

        private void RenderGrid_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isDragging)
            {
                Point currentPos = e.GetPosition(RenderGrid);
                Vector delta = currentPos - _lastMousePosition;

                _panOffset.X += delta.X;
                _panOffset.Y += delta.Y;

                _lastMousePosition = currentPos;

                if (_focusTarget != "Free")
                {
                    CameraFocusComboBox.SelectedIndex = 3;
                }
            }
        }

        private void RenderGrid_MouseUp(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Right || e.ChangedButton == MouseButton.Middle)
            {
                _isDragging = false;
                RenderGrid.ReleaseMouseCapture();
            }
        }

        private void RenderGrid_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            double zoomFactor = e.Delta > 0 ? 1.15 : 0.85;
            _zoomLevel = Math.Clamp(_zoomLevel * zoomFactor, 0.1, 50.0);
        }

        private void CameraFocusComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (CameraFocusComboBox.SelectedItem is ComboBoxItem item && item.Tag is string tag)
            {
                _focusTarget = tag;
                if (_focusTarget != "Free")
                {
                    _panOffset = new Point(0, 0);
                }
            }
        }

        private void ResetCamera_Click(object sender, RoutedEventArgs e)
        {
            _zoomLevel = 1.0;
            _panOffset = new Point(0, 0);
            CameraFocusComboBox.SelectedIndex = 0;
        }

        private void UpdateMetrics()
        {
            _frameCount++;
            double currentTime = _fpsStopwatch.Elapsed.TotalSeconds;

            if (currentTime - _lastFpsTime >= 0.5)
            {
                double fps = _frameCount / (currentTime - _lastFpsTime);
                FpsTextBlock.Text = $"{fps:F1}";

                _frameCount = 0;
                _lastFpsTime = currentTime;
            }

            if (currentTime - _lastUpsTime >= 0.5)
            {
                double deltaTime = currentTime - _lastUpsTime;

                long currentTicks = _physicalEngine.TotalTicks;
                long ticksDelta = currentTicks - _lastEngineTicks;

                double realUps = ticksDelta / deltaTime;

                if (_physicalEngine.TargetUps != 1 && _physicalEngine.IsRunning)
                {
                    double realMultiplier = realUps / 60.0;
                    string multiplierText = realMultiplier < 10.0 ? $"{realMultiplier:F1}x" : $"{realMultiplier:N0}x";
                    UpsTextBlock.Text = $"{realUps:N0} ({multiplierText})";
                }
                else
                {
                    UpsTextBlock.Text = $"{realUps:N0}";
                }

                _lastEngineTicks = currentTicks;
                _lastUpsTime = currentTime;
            }

            double totalSimSeconds = _physicalEngine.TotalTicks * _physicalEngine.FixedDeltaTime.TotalSeconds;
            SimTimeTextBlock.Text = FormatSimTime(totalSimSeconds);
        }

        private string FormatSimTime(double totalSeconds)
        {
            TimeSpan timeSpan = TimeSpan.FromSeconds(totalSeconds);
            double totalDays = timeSpan.TotalDays;

            if (totalDays >= 365.25)
            {
                double years = totalDays / 365.25;
                double remainingDays = totalDays % 365.25;
                return $"{years:N1}y {remainingDays:F0}d {timeSpan.Hours:D2}:{timeSpan.Minutes:D2}:{timeSpan.Seconds:D2}";
            }

            if (totalDays >= 1.0)
            {
                return $"{(int)totalDays}d {timeSpan.Hours:D2}:{timeSpan.Minutes:D2}:{timeSpan.Seconds:D2}";
            }

            return $"{timeSpan.Hours:D2}:{timeSpan.Minutes:D2}:{timeSpan.Seconds:D2}";
        }

        private void UpdateDebugInfo()
        {
            var objects = _simObjectHandler.Objects;
            var sb = new StringBuilder();

            string[] planetNames = { "Sun", "Mercury", "Venus", "Earth", "Mars", "Jupiter", "Saturn", "Uranus", "Neptune" };

            sb.AppendLine($"[Status: {(_physicalEngine.IsRunning ? "Running" : "Paused")}]");
            sb.AppendLine($"Target UPS: {_physicalEngine.TargetUps}");
            sb.AppendLine($"Zoom: {_zoomLevel:F2}x | Camera: {_focusTarget}");
            sb.AppendLine($"Active Maneuvers: {_activeManeuverNodes.Count}");
            sb.AppendLine(new string('-', 30));

            for (int i = 0; i < objects.Length; i++)
            {
                var obj = objects[i];
                string name = i < planetNames.Length ? planetNames[i] : $"Obj {i}";

                sb.AppendLine($"[{i}] {name}:");
                sb.AppendLine($" Pos: ({obj.Position.X.ToDouble():E2}, {obj.Position.Y.ToDouble():E2})");
                sb.AppendLine($" Vel: ({obj.Velocity.X.ToDouble():F1}, {obj.Velocity.Y.ToDouble():F1}) m/s");
                sb.AppendLine();
            }

            DebugTextBlock.Text = sb.ToString();
        }

        private void ToggleSimulationButton_Click(object sender, RoutedEventArgs e)
        {
            if (_physicalEngine.IsRunning)
            {
                _physicalEngine.Stop();
                ToggleSimulationButton.Content = "Старт";
                ToggleSimulationButton.Background = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#28A745"));
            }
            else
            {
                _physicalEngine.Start();
                ToggleSimulationButton.Content = "Пауза";
                ToggleSimulationButton.Background = new SolidColorBrush((Color)ColorConverter.ConvertFromString("#007ACC"));
            }
        }

        private void SpeedComboBox_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (_physicalEngine != null && SpeedComboBox.SelectedItem is ComboBoxItem selectedItem)
            {
                if (selectedItem.Tag is string tagValue && int.TryParse(tagValue, out int targetUps))
                {
                    _physicalEngine.TargetUps = targetUps;
                }
            }
        }

        private void CalculateGravityAssist_Click(object sender, RoutedEventArgs e)
        {
            var planets = _simObjectHandler.Objects;
            if (planets.Length < 6) return;

            int originIndex = GetSelectedPlanetIndex(OriginPlanetComboBox, 3);
            int flybyIndex = GetSelectedPlanetIndex(FlybyPlanetComboBox, 4);
            int targetIndex = GetSelectedPlanetIndex(TargetPlanetComboBox, 5);

            if (originIndex == flybyIndex || flybyIndex == targetIndex || originIndex == targetIndex)
            {
                MessageBox.Show("Выберите три Разные планеты для создания маршрута.", "Ошибка выбора", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            long currentTick = _physicalEngine.TotalTicks;
            TimeSpan dtSeconds = _physicalEngine.FixedDeltaTime;

            PlannerStatusTextBlock.Text = "Оптимизация окна вылета...";
            PlannerStatusTextBlock.Foreground = Brushes.Yellow;
            CalculatePlanButton.IsEnabled = false;

            Task.Run(() =>
            {
                var result = GravityAssistPlanner.FindOptimalFlybyPlan(
                    planets,
                    originIndex: originIndex,
                    flybyIndex: flybyIndex,
                    targetIndex: targetIndex,
                    startTick: currentTick + 10,
                    dtSeconds: EDecimal.FromDouble(dtSeconds.TotalSeconds)
                );

                Dispatcher.InvokeAsync(() =>
                {
                    CalculatePlanButton.IsEnabled = true;

                    if (result.Success)
                    {
                        _activeManeuverNodes.Clear();

                        _activeManeuverNodes.Add(new ManeuverNode(result.DepartureTick, result.DepartureDeltaV));

                        if (!result.FlybyBurnDeltaV.Magnitude.IsZero)
                        {
                            _activeManeuverNodes.Add(new ManeuverNode(result.FlybyTick, result.FlybyBurnDeltaV));
                        }

                        System.Diagnostics.Debug.WriteLine($"[UI] Current Engine Tick: {_physicalEngine.TotalTicks}");
                        System.Diagnostics.Debug.WriteLine($"[UI] Node #1 TargetTick:  {_activeManeuverNodes[0].TargetTick}");
                        System.Diagnostics.Debug.WriteLine($"[UI] Node #2 TargetTick:  {_activeManeuverNodes[1].TargetTick}");

                        PlannerStatusTextBlock.Text = "Оптимальный маневр рассчитан.";
                        PlannerStatusTextBlock.Foreground = Brushes.Lime;

                        var sb = new StringBuilder();
                        sb.AppendLine($"  Маршрут: Obj[{originIndex}] ➔ Obj[{flybyIndex}] ➔ Obj[{targetIndex}]");
                        sb.AppendLine(new string('-', 34));
                        sb.AppendLine($"  Отбытие:");
                        sb.AppendLine($"  Тик: {result.DepartureTick}");
                        sb.AppendLine($"  ΔV1: {result.DepartureDeltaV.Magnitude.ToDouble():F1} м/с");
                        sb.AppendLine();
                        sb.AppendLine($"  Пролет (Flyby):");
                        sb.AppendLine($"  Тик: {result.FlybyTick}");
                        sb.AppendLine($"  Перицентр rp: {result.FlybyPeriapsisKm.ToDouble():N0} км");
                        sb.AppendLine($"  ΔV burn: {result.FlybyBurnDeltaV.Magnitude.ToDouble():F1} м/с");
                        sb.AppendLine();
                        sb.AppendLine($"  Прибытие:");
                        sb.AppendLine($"  Тик: {result.ArrivalTick}");
                        sb.AppendLine(new string('-', 34));
                        sb.AppendLine($"  Суммарный ΔV: {result.TotalDeltaV.ToDouble():F1} м/с");

                        PlanDetailsTextBlock.Text = sb.ToString();
                    }
                    else
                    {
                        PlannerStatusTextBlock.Text = ".Подходящее окно не найдено";
                        PlannerStatusTextBlock.Foreground = Brushes.OrangeRed;
                        PlanDetailsTextBlock.Text = "Не удалось найти окно пролета. Измените комбинацию планет или подождите продвижения времени.";
                    }
                });
            });
        }

        private void ClearManeuvers_Click(object sender, RoutedEventArgs e)
        {
            _activeManeuverNodes.Clear();
            PlannerStatusTextBlock.Text = "Маневры сброшены";
            PlannerStatusTextBlock.Foreground = Brushes.LightGray;
            PlanDetailsTextBlock.Text = "Выберите планеты и нажмите 'Рассчитать маршрут'.";
        }

        private int GetSelectedPlanetIndex(ComboBox comboBox, int defaultIndex)
        {
            if (comboBox.SelectedItem is ComboBoxItem item && item.Tag is string tagStr && int.TryParse(tagStr, out int idx))
            {
                return idx;
            }
            return defaultIndex;
        }

        private void MainWindow_Closing(object? sender, System.ComponentModel.CancelEventArgs e)
        {
            CompositionTarget.Rendering -= OnRenderFrame;
            _physicalEngine.Stop();
        }
    }
}