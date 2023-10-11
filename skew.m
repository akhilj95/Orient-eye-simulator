function omega_wedge = skew(omega)

omega_wedge = [0 -omega(3) omega(2) ; omega(3) 0 -omega(1); -omega(2) omega(1) 0];