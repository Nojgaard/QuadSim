using System.Collections.Generic;
using UnityEngine;

namespace Assets
{
    [RequireComponent(typeof(SimplestPlot))]
    public class QuadPlot : MonoBehaviour
    {
        private class CircularBuffer
        {
            private Queue<float> _queue;
            private int _size;

            public CircularBuffer(int size)
            {
                _queue = new();
                _size = size;
            }

            public void Add(float value)
            {
                _queue.Enqueue(value);
                if (_queue.Count > _size)
                    _queue.Dequeue();
            }

            public float[] Data() => _queue.ToArray();
        }

        public enum PlotType
        {
            MotorSpeed,
            AngularVelocity
        }

        public int NumDataPoints = 300;
        public PlotType Type;

        private SimplestPlot _simplestPlotScript;
        private Color[] _colors;
        private CircularBuffer[] _data;
        private CircularBuffer _timestamps;
        private float _currentTime = 0;

        private QuadSim _quadsim;

        public void Start()
        {
            _simplestPlotScript = GetComponent<SimplestPlot>();
            _colors = new Color[4] { Color.red, Color.green, Color.blue, Color.magenta };
            _data = new CircularBuffer[4];
            _timestamps = new CircularBuffer(NumDataPoints);
            for (int i = 0; i < _data.Length; i++)
                _data[i] = new CircularBuffer(NumDataPoints);

            for (int i = 0; i < _colors.Length; i++)
            {
                _simplestPlotScript.SeriesPlotY.Add(new SimplestPlot.SeriesClass());
                _simplestPlotScript.SeriesPlotY[i].MyColor = _colors[i];
            }

            _quadsim = GameObject.Find("Quadcopter").GetComponentInChildren<QuadSim>();
        }

        public void RecordState(Quadcopter quadcopter, float dt)
        {
            _currentTime += dt;
            _timestamps.Add(_currentTime);
            switch(Type)
            {
                case PlotType.MotorSpeed:
                    for (int i = 0; i < 4; i++)
                        _data[i].Add(quadcopter.MotorAngularVelocity[i]);
                    break;
                case PlotType.AngularVelocity:
                    for (int i = 0; i < 3; i++)
                        _data[i].Add(quadcopter.AngularVelocity[i]);
                    _data[3].Add(0);
                    break;
            }
        }

        public void Update()
        {
            RecordState(_quadsim.quadcopter, Time.deltaTime * _quadsim.timeScale);

            _simplestPlotScript.SeriesPlotX = _timestamps.Data();
            if (_simplestPlotScript.SeriesPlotX.Length == 0)
                return;

            for (int i = 0; i < _simplestPlotScript.SeriesPlotY.Count; i++)
            {
                _simplestPlotScript.SeriesPlotY[i].YValues = _data[i].Data();
            }
            _simplestPlotScript.UpdatePlot();
        }
    }
}
