pub trait InvSqrt {
    fn inv_sqrt(self) -> Self;
}

impl InvSqrt for f32 {
    fn inv_sqrt(self) -> Self {
		const THREEHALFS: f32 = 1.5f32;
		let x2: f32 = self * 0.5f32;
        
        // evil floating point bit level hacking
		let mut i: u32 = unsafe { std::mem::transmute(self) };

        // what the fuck?
		i = 0x5f375a86 - (i >> 1);
		let mut y: f32 = unsafe { std::mem::transmute(i) };

        // 1st iteration
		y = y * ( THREEHALFS - ( x2 * y * y ) );

        // 2nd iteration, this can be removed
//      y = y * ( THREEHALFS - ( x2 * y * y ) );

		return y;
    }
}