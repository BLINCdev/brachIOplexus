using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using XInputDotNetPure;

namespace brachIOplexus
{
    public class ReporterState
    {
        static PlayerIndex[] playerIndices = new PlayerIndex[] { PlayerIndex.One, PlayerIndex.Two, PlayerIndex.Three, PlayerIndex.Four };

        GamePadState[] gamePadStates = new GamePadState[4];
        uint[] lastPacketNumbers = new uint[4];
        int lastActivePlayerIndex = 0;

        public GamePadDeadZone DeadZone { get; set; }
        public int LastActiveIndex { get { return lastActivePlayerIndex; } }
        public GamePadState LastActiveState { get { return gamePadStates[lastActivePlayerIndex]; } }
        public bool LinkTriggersToVibration { get; set; }
        public bool ActivateVibration { get; set; }

        public ReporterState()
        {
            DeadZone = GamePadDeadZone.IndependentAxes;
        }

        public bool Poll()
        {
            for (int i = 0; i < 4; i++)
            {
                gamePadStates[i] = GamePad.GetState(playerIndices[i], DeadZone);
            }

            bool changed = true;

            int activePlayerIndex = lastActivePlayerIndex;
            for (int i = 0; i < 4; i++)
            {
                if (gamePadStates[i].PacketNumber != lastPacketNumbers[i])
                {
                    activePlayerIndex = i;
                    lastPacketNumbers[i] = gamePadStates[i].PacketNumber;
                    changed = true;
                }
            }

            lastActivePlayerIndex = activePlayerIndex;

            if (LinkTriggersToVibration)
            {
                GamePad.SetVibration(playerIndices[lastActivePlayerIndex], LastActiveState.Triggers.Left, LastActiveState.Triggers.Right);
            }
            else
            {
                GamePad.SetVibration(playerIndices[lastActivePlayerIndex], 0.0f, 0.0f);
            }

            // BLINCdev added this code below
            if (ActivateVibration)
            {
                // vibration speeds vary from 0 to 1
                GamePad.SetVibration(playerIndices[lastActivePlayerIndex], 0.0f, 1.0f);
            }
            else
            {
                GamePad.SetVibration(playerIndices[lastActivePlayerIndex], 0.0f, 0.0f);
            }
            // BLINCdev added the code above


            return changed;
        }
    }
}
