impl < __Context > :: bincode :: Decode < __Context > for ControlStepMsg
{
    fn decode < __D : :: bincode :: de :: Decoder < Context = __Context > >
    (decoder : & mut __D) ->core :: result :: Result < Self, :: bincode ::
    error :: DecodeError >
    {
        core :: result :: Result ::
        Ok(Self
        {
            cmd_torque_mask : :: bincode :: Decode :: decode(decoder) ?,
            cmd_raw_target_rad : :: bincode :: Decode :: decode(decoder) ?,
            telem_seq_ack : :: bincode :: Decode :: decode(decoder) ?,
            telem_uptime_ms : :: bincode :: Decode :: decode(decoder) ?,
            telem_active_mode : :: bincode :: Decode :: decode(decoder) ?,
            telem_faults_bits : :: bincode :: Decode :: decode(decoder) ?,
            telem_torque_state_mask : :: bincode :: Decode :: decode(decoder)
            ?, telem_bus_voltage_mv : :: bincode :: Decode :: decode(decoder)
            ?, telem_measured_rad : :: bincode :: Decode :: decode(decoder) ?,
            telem_applied_target_rad : :: bincode :: Decode :: decode(decoder)
            ?, telem_current_ma : :: bincode :: Decode :: decode(decoder) ?,
            telem_temp_c : :: bincode :: Decode :: decode(decoder) ?,
        })
    }
} impl < '__de, __Context > :: bincode :: BorrowDecode < '__de, __Context >
for ControlStepMsg
{
    fn borrow_decode < __D : :: bincode :: de :: BorrowDecoder < '__de,
    Context = __Context > > (decoder : & mut __D) ->core :: result :: Result <
    Self, :: bincode :: error :: DecodeError >
    {
        core :: result :: Result ::
        Ok(Self
        {
            cmd_torque_mask : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, cmd_raw_target_rad : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            telem_seq_ack : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, telem_uptime_ms : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            telem_active_mode : :: bincode :: BorrowDecode ::< '_, __Context
            >:: borrow_decode(decoder) ?, telem_faults_bits : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            telem_torque_state_mask : :: bincode :: BorrowDecode ::< '_,
            __Context >:: borrow_decode(decoder) ?, telem_bus_voltage_mv : ::
            bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, telem_measured_rad : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            telem_applied_target_rad : :: bincode :: BorrowDecode ::< '_,
            __Context >:: borrow_decode(decoder) ?, telem_current_ma : ::
            bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, telem_temp_c : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
        })
    }
}