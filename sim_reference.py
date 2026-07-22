def cascaded2243_offsets_corrected(f0: float = 77e9) -> tuple[np.ndarray, np.ndarray]:

    l = C / f0  # wavelength ~ 3.9 mm at 77 GHz

    ## Use RX 13 as origin point, so that the virtual ULA is centred at x=0.
    tx_dsp = np.array([
        ## AWR #1
        [-(15+11) * l / 2,     -(34-6) * l / 2, 0],
        [-(15+10) * l / 2,     -(34-5) * l / 2, 0],
        [-(15+9) * l / 2,     -(34-1) * l / 2, 0],
        ## AWR #2
        [-(15+32) * l / 2,     -(34) * l / 2, 0],
        [-(15+28) * l / 2,     -(34) * l / 2, 0],
        [-(15+24) * l / 2,     -(34) * l / 2, 0],
        ## AWR #3
        [-(15+20) * l / 2,     -(34) * l / 2, 0],
        [-(15+16) * l / 2,     -(34) * l / 2, 0],
        [-(15+12) * l / 2,     -(34) * l / 2, 0],
        ## AWR #4
        [-(15+8) * l / 2,     -(34) * l / 2, 0],
        [-(15+4) * l / 2,     -(34) * l / 2, 0],
        [-(15+0) * l / 2,     -(34) * l / 2, 0],
    ], dtype=np.float32)
    rx_dsp = np.array([
        ## AWR #1
        [-11 * l / 2, 0, 0],
        [-12 * l / 2, 0, 0],
        [-13 * l / 2, 0, 0],
        [-14 * l / 2, 0, 0],
        ## AWR #2
        [-50 * l / 2, 0, 0],
        [-51 * l / 2, 0, 0],
        [-52 * l / 2, 0, 0],
        [-53 * l / 2, 0, 0],
        ## AWR #3
        [-46 * l / 2, 0, 0],
        [-47 * l / 2, 0, 0],
        [-48 * l / 2, 0, 0],
        [-49 * l / 2, 0, 0],
        ## AWR #4
        [-0 * l / 2, 0, 0],
        [-1 * l / 2, 0, 0],
        [-2 * l / 2, 0, 0],
        [-3 * l / 2, 0, 0],

    ], dtype=np.float32)
    offset_to_rx12 = 25*l

    tx_dsp[:, 0] += offset_to_rx12
    rx_dsp[:, 0] += offset_to_rx12

    return tx_dsp, rx_dsp
