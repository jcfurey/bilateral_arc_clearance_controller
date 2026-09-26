# ROS 2 Lyrical and Jazzy / Nav2 build verification

This container provides a clean build-and-test environment for the complete
package, including the `nav2_core::Controller` plugin, on either of the two
ROS 2 distributions the hosted CI builds: Lyrical (Nav2 1.5, Ubuntu 26.04) and
Jazzy (Nav2 1.3, Ubuntu 24.04). It follows the same `ros:<distro>-ros-base`
image and `rosdep` dependency resolution as the CI. It is intentionally
separate from the ROS 2 Humble Gazebo evidence container, which builds only
the framework-independent core and filter node.

From the package root, run:

```bash
./docker/nav2/verify.sh            # ROS 2 Lyrical
./docker/nav2/verify.sh jazzy      # ROS 2 Jazzy
```

The distribution can also be given as `BAC_ROS_DISTRO`. The script builds
`bac-nav2-<distro>-verification`, mounts the checkout read-only, and performs
a Release `colcon build`, all package tests, and an installed Nav2
plugin-description check in a temporary container workspace. No `build`,
`install`, or `log` directories are written into the checkout.

It then verifies, against the installed package, what the suite above cannot
fail on its own:

- the installed documentation's relative links resolve,
- the tests labelled `ackermann` exist and pass,
- the installed `config/bac_controller_ackermann.yaml` selects the model,
- an installed node accepts a valid Ackermann configuration and stays up,
- it rejects an unsupported `motion_model.type` and a non-positive
  `turn_radius_min` instead of silently falling back to differential drive,
- the installed `bac_filter.launch.py` loads and brings the filter node up on
  the distribution's Python (3.12 on Jazzy, 3.14 on Lyrical), and
- the same three checks for the holonomic model and `limits.vy_max`.

The plugin implements the `nav2_core::Controller` interface of both Nav2
versions; the build selects it from the installed `nav2_core` version (see
`BAC_NAV2_API` in `CMakeLists.txt`). Under Nav2 1.5 the plan handling is
unchanged: the plugin keeps the raw global plan from `newPathReceived()`,
transforms it through TF and prunes it to `max_range` itself, and does not
consume the controller server's path-handler plan. The
[Nav2 integration guide](../../docs/en/nav2_integration.md) states the reason.

Set `BAC_NAV2_IMAGE` to use another local image tag:

```bash
BAC_NAV2_IMAGE=my-bac-lyrical ./docker/nav2/verify.sh
```

## 日本語

このコンテナは、`nav2_core::Controller`プラグインを含むパッケージ全体を、hosted CIが
ビルドする2つのROS 2ディストリビューション、Lyrical（Nav2 1.5、Ubuntu 26.04）と
Jazzy（Nav2 1.3、Ubuntu 24.04）のクリーンな環境でビルド・テストするためのものです。
hosted CIと同じ`ros:<distro>-ros-base`イメージおよび`rosdep`による依存解決を使用します。
ROS 2 HumbleのGazebo evidence環境はcoreとfilter nodeのみを対象とするため、用途を分けています。

パッケージルートで`./docker/nav2/verify.sh`（Lyrical）または`./docker/nav2/verify.sh jazzy`
（Jazzy）を実行すると、ソースをread-onlyでmountし、Releaseビルド、全テスト、インストール済み
plugin descriptionの存在確認を一時コンテナ内で行います。ディストリビューションは
`BAC_ROS_DISTRO`でも指定できます。checkoutに`build`、`install`、`log`は生成しません。

続けて、インストール済みパッケージに対して、上記のテスト一式だけでは検出できない項目を
検証します。インストール済み文書の相対リンク、`ackermann`ラベルのテストが存在して通ること、
インストール済み`config/bac_controller_ackermann.yaml`がモデルを選択していること、実ノードが
妥当なAckermann設定で起動を維持すること、未対応の`motion_model.type`や非正の`turn_radius_min`
を差動二輪へ暗黙にfallbackせず拒否すること、インストール済み`bac_filter.launch.py`がその
ディストリビューションのPython（Jazzyは3.12、Lyricalは3.14）でfilter nodeを起動すること、
そして全方向モデルと`limits.vy_max`についての同種の確認です。

pluginは両Nav2バージョンの`nav2_core::Controller`インターフェースを実装し、ビルド時に
インストール済み`nav2_core`のバージョンから選択します（`CMakeLists.txt`の`BAC_NAV2_API`）。
Nav2 1.5でもplanの扱いは同じで、`newPathReceived()`で受け取った生のglobal planをTFで変換し
`max_range`で自ら刈り込みます。controller serverのpath handlerが渡すplanは使用しません。
理由は[Nav2統合ガイド](../../docs/nav2_integration.md)に記載しています。
