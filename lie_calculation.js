// log map for so3 - rotation matrix
// calculation depend on appendix
// the returned result is a Matrix3
function logarithm_map_so(r){

  var r = r.clone();
  // trace of r
  var trR = r.elements[0] + r.elements[4] + r.elements[8];

	// theta == 0 ?
	var theta = Math.acos((trR - 1)/2);
  //sin theta
  var stheta = Math.sin(theta);

	// R transpose
	var rt = r.clone();
  rt.transpose();

  // calculate r - rt
  var r_arr = r.toArray();
  var rt_arr = rt.toArray();
  var r_sub_rt_arr = [];
  var sum_r_sub_rt = 0;

  for (var i = 0; i < r_arr.length; i++) {
    var val = r_arr[i] - rt_arr[i];
    r_sub_rt_arr.push(val);
    sum_r_sub_rt += val * val;
  }

  var logR = new THREE.Matrix3();
  if (Math.abs(sum_r_sub_rt) < 1e-6) {
    logR.set(0, -1, 1, 1, 0, -1, -1, 1, 0);
    logR.multiplyScalar(1e-4);
  } else {
    logR.fromArray(r_sub_rt_arr);
    logR.multiplyScalar( theta / (2 * stheta) );
  }
  return logR;
}

// expo map for so - rotation matrix
function exponential_map_so(w){

  var o = w.clone();

	var o_s = o.clone();
  o_s.multiply(o_s);
	var tro_s = o_s.elements[0] + o_s.elements[4] + o_s.elements[8];

	var theta = Math.sqrt(-tro_s/2);
  var stheta = Math.sin(theta);


	// O
	var o1 = new THREE.Matrix3();
	var o2 = o.clone();
  o2.multiplyScalar( stheta/ theta);
	var o3 = o_s.clone();
  o3.multiplyScalar( (1 - Math.cos(theta)) / (theta * theta));

  var o1_arr = o1.toArray();
  var o2_arr = o2.toArray();
  var o3_arr = o3.toArray();
  var o_arr = [];

  for (var i = 0; i < o1_arr.length; i++) {
    o_arr.push(o1_arr[i] + o2_arr[i] + o3_arr[i]);
  }

  var o_exp = new THREE.Matrix3();

  // if theta < 1e-6, return the indentity matrix
  if (Math.abs(theta) > 1e-6) {
    o_exp.fromArray(o_arr);
  }

  return o_exp;
}

// log map for se3 - rigid transform
function logarithm_map_se(se){
  var expW = new THREE.Matrix3();
  expW.setFromMatrix4(se);

  var t = new THREE.Vector3(se.elements[12], se.elements[13], se.elements[14]);
  var tr_expW = expW.elements[0] + expW.elements[4] + expW.elements[8];

  var theta = Math.acos((tr_expW - 1)/ 2);
  var stheta = Math.sin(theta);

  var w = logarithm_map_so(expW);
  // var w_vec = new THREE.Vector3(w.elements[5], w.elements[6], w.elements[1]);
  // var w_mat = new THREE.Matrix3();
  // w_mat.set(0, -w_vec.z, w_vec.y, w_vec.z, 0, -w_vec.x, -w_vec.y, w_vec.x, 0);


  // V
  var v1 = new THREE.Matrix3();
  var v2 = w.clone();
  v2.multiplyScalar((1- Math.cos(theta)) / (theta*theta));

  var v3 = w.clone();
  v3.multiply(w.clone());
  v3.multiplyScalar((theta - stheta) / (theta*theta*theta));

  var v1_arr = v1.toArray();
  var v2_arr = v2.toArray();
  var v3_arr = v3.toArray();
  var v_arr = [];

  for (var i = 0; i < v1_arr.length; i++) {
    v_arr.push(v1_arr[i] + v2_arr[i] + v3_arr[i]);
  }

  var v = new THREE.Matrix3();
  v.fromArray(v_arr);

  var v_inv = new THREE.Matrix3();

  // only change when theta is not 0
  if (Math.abs(theta) > 1e-6) {
    v_inv.getInverse(v);
  }

  t.applyMatrix3(v_inv);

  var res = new THREE.Matrix4();

  res.makeTranslation(t.x, t.y, t.z);

  res.elements[0] = w.elements[0];
  res.elements[1] = w.elements[1];
  res.elements[2] = w.elements[2];

  res.elements[4] = w.elements[3];
  res.elements[5] = w.elements[4];
  res.elements[6] = w.elements[5];

  res.elements[8] = w.elements[6];
  res.elements[9] = w.elements[7];
  res.elements[10] = w.elements[8];

  res.elements[15] = 0;

  return res;
}

// exp map for se - rigid motion
function exponential_map_se(se){

  var w = new THREE.Matrix3();
  w.setFromMatrix4(se);

  var t = new THREE.Vector3(se.elements[12], se.elements[13], se.elements[14]);

  // w_s for w square
  var w_s = w.clone();
  w_s.multiply(w.clone());

  var tr_ws = w_s.elements[0] + w_s.elements[4] + w_s.elements[8];

  var theta = Math.sqrt(-tr_ws/2);
  var stheta = Math.sin(theta);

  var w1 = new THREE.Matrix3();
  var w2 = w.clone();
  w2.multiplyScalar( stheta/ theta);
  var w3 = w_s.clone();
  w3.multiplyScalar( (1 - Math.cos(theta)) / (theta * theta));

  var w1_arr = w1.toArray();
  var w2_arr = w2.toArray();
  var w3_arr = w3.toArray();
  var w_arr = [];

  for (var i = 0; i < w1_arr.length; i++) {
    w_arr.push(w1_arr[i] + w2_arr[i] + w3_arr[i]);
  }

  var w_exp = new THREE.Matrix3();
  w_exp.fromArray(w_arr);

  // V
	var v1 = new THREE.Matrix3();
	var v2 = w.clone();
  v2.multiplyScalar((1 - Math.cos(theta)) / (theta * theta));
	var v3 = w_s.clone();
  v3.multiplyScalar((theta - stheta)/ (theta * theta * theta));

  var v1_arr = v1.toArray();
  var v2_arr = v2.toArray();
  var v3_arr = v3.toArray();
  var v_arr = [];

  for (var i = 0; i < v1_arr.length; i++) {
    v_arr.push(v1_arr[i] + v2_arr[i] + v3_arr[i]);
  }

  var v = new THREE.Matrix3();
  v.fromArray(v_arr);


  // if theta is 0, do not need the calculations above
  if (Math.abs(theta) < 1e-6) {
    w_exp = new THREE.Matrix3();
    v = new THREE.Matrix3();
  }

  t.applyMatrix3(v);
  var res = new THREE.Matrix4();

  res.makeTranslation(t.x, t.y, t.z);

  res.elements[0] = w_exp.elements[0];
  res.elements[1] = w_exp.elements[1];
  res.elements[2] = w_exp.elements[2];

  res.elements[4] = w_exp.elements[3];
  res.elements[5] = w_exp.elements[4];
  res.elements[6] = w_exp.elements[5];

  res.elements[8] = w_exp.elements[6];
  res.elements[9] = w_exp.elements[7];
  res.elements[10] = w_exp.elements[8];

  return res;

}


// log map for scale - scale matrix
function logarithm_map_sc(sc){

  var x = Math.log(sc.elements[0]);
  var y = Math.log(sc.elements[5]);
  var z = Math.log(sc.elements[10]);
  var w = Math.log(sc.elements[15]);

  var res = new THREE.Matrix4();
  res.elements[0] = x;
  res.elements[5] = y;
  res.elements[10] = z;
  res.elements[15] = w;

  return res;
}


function exponential_map_sc(sc){

  var x = Math.exp(sc.elements[0]);
  var y = Math.exp(sc.elements[5]);
  var z = Math.exp(sc.elements[10]);
  var w = Math.exp(sc.elements[15]);

  var res = new THREE.Matrix4();
  res.elements[0] = x;
  res.elements[5] = y;
  res.elements[10] = z;
  res.elements[15] = w;

  return res;
}
