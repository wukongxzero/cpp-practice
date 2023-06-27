#include <iostream>
#include <vector>

using namespace std;

int main() {
  // Get the length of 'a' and the number of queries
  int n, q;
  cin >> n >> q;

  // Create a vector of vectors
  vector<vector<int>> a;
  a.reserve(n);

  // Get the arrays a[0], a[1], ...
  for (int i = 0; i < n; i++) {
    // Get the length of the vector at a[i]
    int k;
    cin >> k;
    vector<int> arr(k);

    for (int j = 0; j < k; j++) {
      cin >> arr[j];
    }

    a.push_back(move(arr));
  }

  // Perform the queries
  for (int q_num = 0; q_num < q; q_num++) {
    int i, j;
    cin >> i >> j;

    // Check if i and j are within valid range
    if (i >= 0 && i < n && j >= 0 && j < a[i].size()) {
      cout << a[i][j] << endl;
    } else {
      cout << "Invalid query!" << endl;
    }
  }

  return 0;
}
