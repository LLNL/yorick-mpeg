/*
 * mpgtest.i -- $Id: mpgtest.i,v 1.1 2007-10-27 22:18:21 dhmunro Exp $
 * test yorick mpeg encoder
 */
require, "mpeg.i";

require, "movie.i";
if (is_void(_orig_movie)) _orig_movie = movie;
require, "demo2.i";

func mpgtest(void)
{
  _mpgtest_name = "test.mpg";
  movie = _mpgtest_movie;
  demo2, 3;
}

func _mpgtest_movie(__f, __a, __b, __c)
{
  movie = _orig_movie;
  return mpeg_movie(_mpgtest_name, __f, __a, __b, __c);
}
