impl < __Context > :: bincode :: Decode < __Context > for HostCmdMsg
{
    fn decode < __D : :: bincode :: de :: Decoder < Context = __Context > >
    (decoder : & mut __D) ->core :: result :: Result < Self, :: bincode ::
    error :: DecodeError >
    {
        core :: result :: Result ::
        Ok(Self
        {
            present : :: bincode :: Decode :: decode(decoder) ?, seq : ::
            bincode :: Decode :: decode(decoder) ?, mode : :: bincode ::
            Decode :: decode(decoder) ?, torque_mask : :: bincode :: Decode ::
            decode(decoder) ?, speed_limit_pct : :: bincode :: Decode ::
            decode(decoder) ?, target_rad : :: bincode :: Decode ::
            decode(decoder) ?, cfg_tag : :: bincode :: Decode ::
            decode(decoder) ?, cfg_joint : :: bincode :: Decode ::
            decode(decoder) ?, cfg_f0 : :: bincode :: Decode ::
            decode(decoder) ?, cfg_f1 : :: bincode :: Decode ::
            decode(decoder) ?, cfg_bool : :: bincode :: Decode ::
            decode(decoder) ?,
        })
    }
} impl < '__de, __Context > :: bincode :: BorrowDecode < '__de, __Context >
for HostCmdMsg
{
    fn borrow_decode < __D : :: bincode :: de :: BorrowDecoder < '__de,
    Context = __Context > > (decoder : & mut __D) ->core :: result :: Result <
    Self, :: bincode :: error :: DecodeError >
    {
        core :: result :: Result ::
        Ok(Self
        {
            present : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, seq : :: bincode :: BorrowDecode ::< '_,
            __Context >:: borrow_decode(decoder) ?, mode : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            torque_mask : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, speed_limit_pct : :: bincode ::
            BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            target_rad : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, cfg_tag : :: bincode :: BorrowDecode ::<
            '_, __Context >:: borrow_decode(decoder) ?, cfg_joint : :: bincode
            :: BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
            cfg_f0 : :: bincode :: BorrowDecode ::< '_, __Context >::
            borrow_decode(decoder) ?, cfg_f1 : :: bincode :: BorrowDecode ::<
            '_, __Context >:: borrow_decode(decoder) ?, cfg_bool : :: bincode
            :: BorrowDecode ::< '_, __Context >:: borrow_decode(decoder) ?,
        })
    }
}