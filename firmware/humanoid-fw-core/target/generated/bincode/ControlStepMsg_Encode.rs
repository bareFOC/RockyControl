impl :: bincode :: Encode for ControlStepMsg
{
    fn encode < __E : :: bincode :: enc :: Encoder >
    (& self, encoder : & mut __E) ->core :: result :: Result < (), :: bincode
    :: error :: EncodeError >
    {
        :: bincode :: Encode :: encode(&self.cmd_torque_mask, encoder) ?; ::
        bincode :: Encode :: encode(&self.cmd_raw_target_rad, encoder) ?; ::
        bincode :: Encode :: encode(&self.telem_seq_ack, encoder) ?; ::
        bincode :: Encode :: encode(&self.telem_uptime_ms, encoder) ?; ::
        bincode :: Encode :: encode(&self.telem_active_mode, encoder) ?; ::
        bincode :: Encode :: encode(&self.telem_faults_bits, encoder) ?; ::
        bincode :: Encode :: encode(&self.telem_torque_state_mask, encoder) ?;
        :: bincode :: Encode :: encode(&self.telem_bus_voltage_mv, encoder) ?;
        :: bincode :: Encode :: encode(&self.telem_measured_rad, encoder) ?;
        :: bincode :: Encode ::
        encode(&self.telem_applied_target_rad, encoder) ?; :: bincode ::
        Encode :: encode(&self.telem_current_ma, encoder) ?; :: bincode ::
        Encode :: encode(&self.telem_temp_c, encoder) ?; core :: result ::
        Result :: Ok(())
    }
}