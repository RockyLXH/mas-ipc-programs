#include <ctime>
#include <iostream>
#include <curses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

// the coordination of screen as below:
// ------------------------> X
// |
// |
// |
// |
// |
// Y

// block layout is: {w-1,h-1},      {x0,y0},        {x1,y1},      {x2,y2},     {x3,y3} (two bits each)
//            bits: {19-18, 17-16}, {15-14, 13-12}, {11-10, 9-8}, {7-6, 5-4}, {3-2, 1-0}
int x = 431424, y = 598356, r = 427089, px = 247872, py = 799248, pr, nr, np,
    c = 348480, p = 615696, tick, board[20][10],
    block[7][4] = {{x, y, x, y},                      // Z
                   {r, p, r, p},                      // S
                   {c, c, c, c},                      // O
                   {599636, 431376, 598336, 432192},  // J
                   {411985, 610832, 415808, 595540},  // T
                   {px, py, px, py},                  // I
                   {614928, 399424, 615744, 428369}}, // L
    score = 0;
  WINDOW *main_window; 
  int Z, S, O, J, T, I, L;

struct Timer
{
	private:
	std::chrono::steady_clock::time_point start;

	public:
	static double freq;
	Timer()
	{
		start = std::chrono::steady_clock::now();
	}
	~Timer()
	{
		auto dur = std::chrono::steady_clock::now() - start;
		freq = 1000000000.0 / dur.count();
	}
};

double Timer::freq = 0;

// extract a 2-bit number from a block entry
int NUM(int x, int y) { return 3 & block[p][x] >> y; }

WINDOW *create_subwin(WINDOW *window, int height, int width, int starty, int startx)
{
    WINDOW *local_win;
    local_win = subwin(window, height, width, starty, startx);
    box(local_win, 0 , 0);      
    return local_win;
}
void next_piece()
{
  np = rand() % 7;
  nr = rand() % 4;
  int _arr[4][4] = {0};

  for (int i = 0, j = 0; i < 8; i += 2, ++j)
  {
    int _x = 3 & block[np][nr] >> (i * 2);
    int _y = 3 & block[np][nr] >> (i * 2 + 2);
    _arr[_x][_y] = np + 1;
  }
  for (int i = 0; i < 4; ++i)
  {
    move(1 + i, 23);
    for (int j = 0; j < 4; ++j)
    {
      _arr[i][j] && attron(262176 | _arr[i][j] << 8); // => if (_arr[i][j] > 0) _arr[i][j] << 8;
      printw("  ");
      attroff(262176 | _arr[i][j] << 8);
    }
  }
}
// create a new piece, don't remove old one (it has landed and should stick)
void new_piece()
{
  switch (p)
  {
    case 0 : ++Z; break;
    case 1 : ++S; break;
    case 2 : ++O; break;
    case 3 : ++J; break;
    case 4 : ++T; break;
    case 5 : ++I; break;
    case 6 : ++L; break;
    default : break;
  }
  y = py = 0;
  p = np;
  r = pr = nr;
  x = px = rand() % (10 - NUM(r, 16)); // get x offset, should be >= block's width in X axis
}

// draw the board and score and fill the color
void frame()
{
  for (int i = 0; i < 20; i++)
  {
    move(1 + i, 1); // otherwise the box won't draw
    for (int j = 0; j < 10; j++)
    {
      board[i][j] && attron(262176 | board[i][j] << 8); // => if (board[i][j] > 0) board[i][j] << 8;
      printw("  ");
      attroff(262176 | board[i][j] << 8);
    }
  }

  static int cnt = 0;
  if (++cnt == 20) {
	  mvprintw(10, 22, "FPS: %0.2f", Timer::freq);
	  cnt = 0;
  }

  mvprintw(20, 22, "Score: %d", score);
  mvprintw(18, 22, "Z: %d", Z);
  mvprintw(17, 22, "S: %d", S);
  mvprintw(16, 22, "O: %d", O);
  mvprintw(15, 22, "J: %d", J);
  mvprintw(14, 22, "T: %d", T);
  mvprintw(13, 22, "I: %d", I);
  mvprintw(12, 22, "L: %d", L);
  refresh();
}

// set the value fo the board for a particular (x,y,r) piece
void set_piece(int x, int y, int r, int v)
{
  for (int i = 0; i < 8; i += 2)
  {
    int _x = NUM(r, i * 2) + y;
    int _y = NUM(r, (i * 2) + 2) + x;
    board[_x][_y] = v;
  }
}

// move a piece from old (p*) coords to new
int update_piece()
{
  set_piece(px, py, pr, 0);
  set_piece(px = x, py = y, pr = r, p + 1);
  return 0;
}

// remove line(s) from the board if they're full
void remove_line()
{
  for (int row = y; row <= y + NUM(r, 18); row++)
  {
    c = 1;
    for (int i = 0; i < 10; i++)
    {
      c *= board[row][i];
    }
    if (!c)
    {
      continue;
    }
    for (int i = row - 1; i > 0; i--)
    {
      memcpy(&board[i + 1][0], &board[i][0], 40);
    }
    memset(&board[0][0], 0, 10);
    score++;
  }
}

// check if placing p at (x,y,r) will be a collision
int check_hit(int x, int y, int r)
{
  if (y + NUM(r, 18) > 19)
  {
    return 1;
  }
  set_piece(px, py, pr, 0);
  c = 0;
  for (int i = 0; i < 8; i += 2)
  {
    board[y + NUM(r, i * 2)][x + NUM(r, (i * 2) + 2)] && c++;
  }
  set_piece(px, py, pr, p + 1);
  return c;
}

// slowly tick the piece y position down so the piece falls
int do_tick()
{
  if (++tick > 30)
  {
    tick = 0;
    if (check_hit(x, y + 1, r))
    {
      if (!y)
      {
        return 0;
      }
      remove_line();
      new_piece();
      next_piece();
    }
    else
    {
      y++;
      update_piece();
    }
  }
  return 1;
}

// main game loop with wasd input checking
void runloop()
{
  while (do_tick())
  {
	  Timer fps;
    usleep(10000);
    if ((c = getch()) == 'a' && x > 0 && !check_hit(x - 1, y, r))
    {
      x--;
    }
    if (c == 'd' && x + NUM(r, 16) < 9 && !check_hit(x + 1, y, r))
    {
      x++;
    }
    if (c == 's')
    {
      while (!check_hit(x, y + 1, r))
      {
        y++;
        update_piece();
      }
      remove_line();
      new_piece();
      next_piece();
    }
    if (c == 'w')
    {
      ++r %= 4;
      while (x + NUM(r, 16) > 9)
      {
        x--;
      }
      if (check_hit(x, y, r))
      {
        x = px;
        r = pr;
      }
    }
    if (c == 'q')
    {
      return;
    }
    update_piece();
    frame();
  }
}


// init curses and start runloop
int main()
{
  srand(time(0));
  initscr();
  start_color();
  // colours indexed by their position in the block
  for (int i = 1; i < 8; i++)
  {
    init_pair(i, i, 0);
  }
  y = py = 0;
  p = rand() % 7;
  r = pr = rand() % 4;
  x = px = rand() % (10 - NUM(r, 16)); // get x offset, should be >= block's width in X axis
  resizeterm(22, 33);
  noecho();
  timeout(0); // set to non block mode, the program continues to run while waitting for user input
  curs_set(0);
  main_window = create_subwin(stdscr, 22, 22, 0, 0);
  box(stdscr, 0, 0);
  next_piece();
  runloop();
  endwin();
}
