impl :: bincode :: Encode for HostCmdMsg
{
    fn encode < __E : :: bincode :: enc :: Encoder >
    (& self, encoder : & mut __E) ->core :: result :: Result < (), :: bincode
    :: error :: EncodeError >
    {
        :: bincode :: Encode :: encode(&self.present, encoder) ?; :: bincode
        :: Encode :: encode(&self.seq, encoder) ?; :: bincode :: Encode ::
        encode(&self.mode, encoder) ?; :: bincode :: Encode ::
        encode(&self.torque_mask, encoder) ?; :: bincode :: Encode ::
        encode(&self.speed_limit_pct, encoder) ?; :: bincode :: Encode ::
        encode(&self.target_rad, encoder) ?; :: bincode :: Encode ::
        encode(&self.cfg_tag, encoder) ?; :: bincode :: Encode ::
        encode(&self.cfg_joint, encoder) ?; :: bincode :: Encode ::
        encode(&self.cfg_f0, encoder) ?; :: bincode :: Encode ::
        encode(&self.cfg_f1, encoder) ?; :: bincode :: Encode ::
        encode(&self.cfg_bool, encoder) ?; core :: result :: Result :: Ok(())
    }
}