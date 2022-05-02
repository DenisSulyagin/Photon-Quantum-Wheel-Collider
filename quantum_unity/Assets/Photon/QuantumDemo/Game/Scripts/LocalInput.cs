using System;
using Photon.Deterministic;
using Quantum;
using UnityEngine;

public class LocalInput : MonoBehaviour
{
    private void OnEnable()
    {
        QuantumCallback.Subscribe(this, (CallbackPollInput callback) => PollInput(callback));
    }

    public void PollInput(CallbackPollInput callback)
    {
        var x = UnityEngine.Input.GetAxis("Horizontal");
        var y = UnityEngine.Input.GetAxis("Vertical");

        var i = new Quantum.Input()
        {
            throttle = Mathf.Clamp01(y).ToFP(),
            brake = Mathf.Clamp01(-y).ToFP(),
            steer = x.ToFP(),
            reset = UnityEngine.Input.GetKeyDown(KeyCode.R)
        };

        callback.SetInput(i, DeterministicInputFlags.Repeatable);
    }
}
