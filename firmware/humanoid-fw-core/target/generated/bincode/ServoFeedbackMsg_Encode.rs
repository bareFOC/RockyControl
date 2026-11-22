impl :: bincode :: Encode for ServoFeedbackMsg
{
    fn encode < __E : :: bincode :: enc :: Encoder >
    (& self, encoder : & mut __E) ->core :: result :: Result < (), :: bincode
    :: error :: EncodeError >
    {
        :: bincode :: Encode :: encode(&self.raw_rad, encoder) ?; :: bincode
        :: Encode :: encode(&self.current_ma, encoder) ?; :: bincode :: Encode
        :: encode(&self.temp_c, encoder) ?; :: bincode :: Encode ::
        encode(&self.bus_voltage_mv, encoder) ?; :: bincode :: Encode ::
        encode(&self.bus_error, encoder) ?; core :: result :: Result :: Ok(())
    }
}