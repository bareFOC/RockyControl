impl < __Context > :: bincode :: Decode < __Context > for ServoFeedbackMsg
{
    fn decode < __D : :: bincode :: de :: Decoder < Context = __Context > >
    (decoder : & mut __D) ->core :: result :: Result < Self, :: bincode ::
    error :: DecodeError >
    {
        core :: result :: Result ::
        Ok(Self
        {
            raw_rad : :: bincode :: Decode :: decode(decoder) ?, current_ma :
            :: bincode :: Decode :: decode(decoder) ?, temp_c : :: bincode ::
            Decode :: decode(decoder) ?, bus_voltage_mv : :: bincode :: Decode
            :: decode(decoder) ?, bus_error : :: bincode :: Decode ::
            decode(decoder) ?,
        })
    }
} impl < '__de, __Context > :: bincode :: BorrowDecode < '__de, __Context >
for ServoFeedbackMsg
{
    fn borrow_decode < __D : :: bincode :: de :: BorrowDecoder < '__de,
    Context = __Context > > (decoder : & mut __D) ->core :: result :: Result <
    Self, :: bincode :: error :: DecodeError >
    {
        core :: result :: Result ::
        Ok(Self
        {
            raw_rad : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, current_ma : :: bincode :: BorrowDecode
            ::< '_, __Context >:: borrow_decode(decoder) ?, temp_c : ::
            bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, bus_voltage_mv : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            bus_error : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?,
        })
    }
}