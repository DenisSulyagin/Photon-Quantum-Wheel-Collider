using System;
using Photon.Deterministic;
using Quantum;
using UnityEngine;

public class LocalInput : MonoBehaviour
{
    public KeyCode throttleKey;
    public KeyCode brakeKey;
    public KeyCode resetKey;

    private void OnEnable()
    {
        QuantumCallback.Subscribe(this, (CallbackPollInput callback) => PollInput(callback));
    }

    public void PollInput(CallbackPollInput callback)
    {
        var i = new Quantum.Input()
        {
            throttle = UnityEngine.Input.GetKey(throttleKey),
            brake = UnityEngine.Input.GetKey(brakeKey),
            steer = UnityEngine.Input.GetAxis("Horizontal").ToFP(),
            reset = UnityEngine.Input.GetKeyDown(resetKey)
        };

        callback.SetInput(i, DeterministicInputFlags.Repeatable);
    }
}
