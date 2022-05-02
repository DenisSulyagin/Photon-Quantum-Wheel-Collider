using System;
using Photon.Deterministic;
using Quantum;
using UnityEngine;

public class LocalInput : MonoBehaviour {
    
  private void OnEnable() {
    QuantumCallback.Subscribe(this, (CallbackPollInput callback) => PollInput(callback));
  }

  public void PollInput(CallbackPollInput callback)
  {
    var x = UnityEngine.Input.GetAxis("Horizontal");
    var y = UnityEngine.Input.GetAxis("Vertical");

    var i = new Quantum.Input()
    {
      Movement = new FPVector2(x.ToFP(), y.ToFP())
    };

    callback.SetInput(i, DeterministicInputFlags.Repeatable);
  }
}
