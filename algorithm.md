input:
  positions p_i for 0 .. n
  joint lengths d_i for 0 .. n
  base position b
  target position t

  total_dist = d_1 + d_2 .. + d_n
  let root_target_dist = t - b . length

  if root_target_dist > total dist {
    // target is unreachable

    for i in 0..n-1 {
      //find the distance r_i between the target and the joint.

      r_i = t - p_i .length
      λ_i = d_i / r_i

      //find the joint position
      p_i+1 = (1 - λ_i)p_i + λt 
    }
  } else {
    //the target is reachable
    p_1 = b

    // create minimum tolerance
    diff_A = (p_n - t).length
    tollerance = f32 (some value)

    while diff_A > tollerance {
      // STAGE 1: FORWARD REACHING
      // Set the end joint to the target
      p_n = t

      for i = n-1..1 {
        //find the distance r_i between the new joint position of p_i+1 and p_i

        r_i = (p_i+1 - p_i).length
        λ_i = d_i / r_i
        //find the new joint position of p_i
        p_i = (1 - λ_i) p_i+1 + λ_i p_i
        
      }
      //STAGE 2: BACKWARD REACHING
      // set p_1 back to b
      p_1 = b

      for i = 1..n - 1 {
        //find the distance r_i between the new joint position p_i
        r_i = (p_i+1 - p_i).length
        λ_i = d_i / r+i
        //find the new position
        p_i+1 = (1- λ_i) p_i + λi p_i+1
      }
      diff_A = (p_n - t).length
    }
  }
