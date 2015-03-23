#ifndef URG_SERIAL_UTILS_H
#define URG_SERIAL_UTILS_H

/*!
  \file
  \brief シリアル用の補助関数

  \author Satofumi KAMIMURA

  $Id: urg_serial_utils.h,v acb362e60f78 2014/09/10 05:36:24 jun $
*/

#ifdef __cplusplus
extern "C" {
#endif

//! シリアルポートを検索する
extern int urg_serial_find_port(void);


//! 検索したシリアルポート名を返す
extern const char *urg_serial_port_name(int index);


/*!
  \brief ポートが URG かどうか

  \retval 1 URG のポート
  \retval 0 不明
  \retval <0 エラー
*/
extern int urg_serial_is_urg_port(int index);

#ifdef __cplusplus
}
#endif

#endif /* !URG_SERIAL_UTILS_H */
