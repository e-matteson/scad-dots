$fn=7;
hull()
{
	translate([0,0,0])
	{
		rotate(0,[0,0,1])
		{
			cube([3,3,3]);
		}
	}
	translate([0,0,7])
	{
		rotate(0,[0,0,1])
		{
			cube([3,3,3]);
		}
	}
}
