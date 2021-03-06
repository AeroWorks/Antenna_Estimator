function [x, y, azi, rk] = eqdazim_fwd(lat0, lon0, lat, lon, ellipsoid)
%EQDAZIM_FWD  Forward azimuthal equidistant projection
%
%   [x, y] = EQDAZIM_FWD(lat0, lon0, lat, lon)
%   [x, y, azi, rk] = EQDAZIM_FWD(lat0, lon0, lat, lon, ellipsoid)
%
%   performs the forward azimuthal equidistant projection of points
%   (lat,lon) to (x,y) using (lat0,lon0) as the center of projection.
%   These input arguments can be scalars or arrays of equal size.  The
%   ellipsoid vector is of the form [a, e], where a is the equatorial
%   radius in meters, e is the eccentricity.  If ellipsoid is omitted, the
%   WGS84 ellipsoid (more precisely, the value returned by
%   defaultellipsoid) is used.  geodproj defines the projection and gives
%   the restrictions on the allowed ranges of the arguments.  The inverse
%   projection is given by eqdazim_inv.
%
%   azi and rk give metric properties of the projection at (lat,lon); azi
%   is the azimuth of the geodesic from the center of projection and rk is
%   the reciprocal of the azimuthal scale.  The scale in the radial
%   direction is 1.
%
%   lat0, lon0, lat, lon, azi are in degrees.  The projected coordinates x,
%   y are in meters (more precisely the units used for the equatorial
%   radius).  rk is dimensionless.
%
%   Section 14 of
%
%     C. F. F. Karney, Geodesics on an ellipsoid of revolution (2011),
%     http://arxiv.org/abs/1102.1215
%     Errata: http://geographiclib.sf.net/geod-addenda.html#geod-errata
%
%   describes how to use this projection in the determination of maritime
%   boundaries (finding the median line).
%
%   See also GEODPROJ, EQDAZIM_INV, GEODDISTANCE, DEFAULTELLIPSOID.

% Copyright (c) Charles Karney (2012-2015) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.42.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    [~] = lat0 + lon0 + lat + lon;
  catch
    error('lat0, lon0, lat, lon have incompatible sizes')
  end

  [s, azi0, azi, ~, m, ~, ~, sig] = ...
      geoddistance(lat0, lon0, lat, lon, ellipsoid);
  azi0 = azi0 * (pi/180);
  x = s .* sin(azi0);
  y = s .* cos(azi0);
  rk = m ./ s;
  rk(sig <= 0.01 * sqrt(realmin)) = 1;
end
